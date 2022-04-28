#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include <driver/gpio.h>
#include <driver/timer.h>
#include <esp_log.h>

#include "button.h"
#include "internal/esp32-button.h"

// DEBT: Either static-ize this or prefix to avoid collisions
extern int pin_count;
extern debounce_t * debounce;
extern QueueHandle_t queue;

static gpio_isr_handle_t gpio_isr_handle;
static timer_isr_handle_t timer_isr_handle;

static unsigned down_events = 0;

static const char* TAG = "esp32 Button ISR";

static int timer_group = TIMER_GROUP_0;

static void update_button_history(debounce_t *d, 
    unsigned time_passage,
    uint32_t level) {
    do {
        d->history <<= 1;
        d->history |= level;
    } while(--time_passage);
}

static void update_button(debounce_t *d, 
    uint32_t level) {
    d->history = (d->history << 1) | level;
}

static void send_event(debounce_t* db, int ev) {
    button_event_t event = {
        .pin = db->pin,
        .event = ev,
    };
    BaseType_t rtosResult = xQueueSendToBackFromISR(queue, &event, NULL);

    // DEBT: Wrap this in DEBUG mode #ifdef
    if(rtosResult == errQUEUE_FULL)
        ets_printf("ISR send_event queue full");
}

#define MASK   0b1111000000111111
static bool button_rose(debounce_t *d) {
    if ((d->history & MASK) == 0b0000000000111111) {
        d->history = 0xffff;
        return 1;
    }
    return 0;
}
static bool button_fell(debounce_t *d) {
    if ((d->history & MASK) == 0b1111000000000000) {
        d->history = 0x0000;
        return 1;
    }
    return 0;
}
static bool button_down(debounce_t *d) {
    if (d->inverted) return button_fell(d);
    return button_rose(d);
}
static bool button_up(debounce_t *d) {
    if (d->inverted) return button_rose(d);
    return button_fell(d);
}

static unsigned threshold_ms = 50;

// FIX: Not valid for multiple buttons and not debouncing anything at all
// Just getting things compiling and running for now
static void gpio_isr(void* context) {
    // Guidance from:
    // https://esp32.com/viewtopic.php?t=345 
    // 
    uint32_t gpio_intr_status = READ_PERI_REG(GPIO_STATUS_REG);   //read status to get interrupt status for GPIO0-31
    uint32_t gpio_intr_status_h = READ_PERI_REG(GPIO_STATUS1_REG);//read status1 to get interrupt status for GPIO32-39
    // Fun fact - your ESP32 will reset if you don't clear your interrupts :)
    SET_PERI_REG_MASK(GPIO_STATUS_W1TC_REG, gpio_intr_status);    //Clear intr for gpio0-gpio31
    SET_PERI_REG_MASK(GPIO_STATUS1_W1TC_REG, gpio_intr_status_h); //Clear intr for gpio32-39

    // https://www.esp32.com/viewtopic.php?t=20123 indicates we can call this here
    uint32_t millis = esp_timer_get_time() / 1000;

    ets_printf("1 gpio Intr\n");

    for (int idx=0; idx<pin_count; idx++) {
        debounce_t* const d =  &debounce[idx];

        if(gpio_intr_status & BIT(d->pin)) { //gpio0-gpio31
            int level = gpio_get_level(d->pin);

            ets_printf("1 Intr GPIO%d, val: %d\n",d->pin, level);

            // We're only looking to debounce the up part here
            // The downpress part we're sniffing in the timer
            if(level == 1) {
                unsigned time_passage;

                if(d->down_time != 0)
                {
                    time_passage = (millis - d->down_time) / threshold_ms;

                    // We toss bounciness in a sub threshold range
                    if(time_passage == 0)
                        continue;
                }
                else
                    time_passage = 1;

                // backfill all the passed time with previous level setting
                if(time_passage > 1) {
                    // We may get way past our window.  Make sure we don't
                    // flip out in that case
                    if(time_passage < 32)
                        update_button_history(d, time_passage, !level);
                    else
                        // FIX: Not correct here
                        update_button_history(d, 32, !level);
                }

                update_button(d, level);
            }
            else {
                // Kick off timer to initiate:
                // - DOWN event once it's been down long enough
                // - HELD event once it's been down that much further
            }

            //d->down_time = millis;
            //++down_events;
            //send_event(d, BUTTON_DOWN);
            timer_start(timer_group,0);
        }

        if (button_up(d)) {
            d->down_time = 0;
            send_event(d, BUTTON_UP);
            /* HELD to be assisted by timer ISR
        } else if (d->down_time && millis() >= debounce[idx].next_long_time) {
            debounce[idx].next_long_time = debounce[idx].next_long_time + CONFIG_ESP32_BUTTON_LONG_PRESS_REPEAT_MS;
            send_event(d, BUTTON_HELD); */
            /* DOWN also to be done by timer ISR
        } else if (button_down(d) && d->down_time == 0) {
            d->down_time = millis;
            d->next_long_time = d->down_time + CONFIG_ESP32_BUTTON_LONG_PRESS_DURATION_MS;
            send_event(d, BUTTON_DOWN); */
        }
    }
}

QueueHandle_t esp32_button_init_internal(
    unsigned long long pin_select,
    gpio_pull_mode_t pull_mode) {
    if (pin_count != -1) {
        ESP_LOGW(TAG, "Already initialized");
        return NULL;
    }

    // Configure the pins
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_INTR_ANYEDGE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = (pull_mode == GPIO_PULLUP_ONLY || pull_mode == GPIO_PULLUP_PULLDOWN);
    io_conf.pull_down_en = (pull_mode == GPIO_PULLDOWN_ONLY || pull_mode == GPIO_PULLUP_PULLDOWN);;
    io_conf.pin_bit_mask = pin_select;
    gpio_config(&io_conf);
    // LEVEL1 is lowest priority, which seems appropriate for human-speed interrupts, even
    // considering the wiggle of the bounce
    
    ESP_ERROR_CHECK(
        gpio_isr_register(gpio_isr, NULL, ESP_INTR_FLAG_LEVEL1, &gpio_isr_handle));

    // Scan the pin map to determine number of pins
    pin_count = 0;
    for (int pin=0; pin<=39; pin++) {
        if ((1ULL<<pin) & pin_select) {
            pin_count++;
        }
    }

    // Initialize global state and queue
    debounce = calloc(pin_count, sizeof(debounce_t));
    queue = xQueueCreate(CONFIG_ESP32_BUTTON_QUEUE_SIZE, sizeof(button_event_t));

    // Scan the pin map to determine each pin number, populate the state
    uint32_t idx = 0;
    for (int pin=0; pin<=39; pin++) {
        if ((1ULL<<pin) & pin_select) {
            ESP_LOGI(TAG, "Registering button input: %d", pin);
            debounce[idx].pin = pin;
            debounce[idx].down_time = 0;
            debounce[idx].inverted = true;
            if (debounce[idx].inverted) debounce[idx].history = 0xffff;
            idx++;
        }
    }

    return queue;
}

static uint32_t ms_test = 0;

// Guidance from
// https://www.esp32.com/viewtopic.php?t=12931 
static void IRAM_ATTR timer_group0_isr (void *param){
    TIMERG0.int_clr_timers.t0 = 1; //clear interrupt bit
    TIMERG0.hw_timer[0].config.alarm_en = 1;    // re-enable alarm

    uint32_t millis = esp_timer_get_time() / 1000;

    if(ms_test == 0) {
        ets_printf("1 timer Intr millis=%u\n", millis);
    }

    ms_test = millis;

    if(down_events > 0) {
        for (int idx=0; idx<pin_count; idx++) {
            debounce_t* const d =  &debounce[idx];
            // TODO: Doesn't handle 0=up yet
            int level = 0;
            update_button(d, level);

            if(button_down(d) && d->down_time == 0) {
                d->down_time = millis;
                d->next_long_time = d->down_time + CONFIG_ESP32_BUTTON_LONG_PRESS_DURATION_MS;
                send_event(d, BUTTON_DOWN);
            }
        }
    }
}

QueueHandle_t button_init_isr(unsigned long long pin_select) {
    QueueHandle_t queue = esp32_button_init_internal(pin_select, GPIO_FLOATING);
    
    timer_config_t timer;
    
    // Set prescaler for 10 KHz clock.  We'd go slower if we could, but:
    // "The divider’s range is from from 2 to 65536."
    timer.divider = 8000; 

    timer.counter_dir = TIMER_COUNT_UP;
    timer.alarm_en = 1;
    timer.intr_type = TIMER_INTR_LEVEL;
    timer.auto_reload = TIMER_AUTORELOAD_EN; // Reset timer to 0 when end condition is triggered
    timer.counter_en = TIMER_PAUSE;

    timer_init(timer_group, 0, &timer);
    timer_set_counter_value(timer_group, 0, 0);
    timer_isr_register(timer_group, 0, timer_group0_isr, NULL, 
        ESP_INTR_FLAG_LEVEL1 | ESP_INTR_FLAG_IRAM,
        &timer_isr_handle);
    
    // Brings us to an alarm every 10ms (100 counting / 10 Khz = 0.01s = 10ms)
    timer_set_alarm_value(timer_group, 0, 100);

    timer_enable_intr(timer_group,0);

    return queue;
}

void button_deinit_isr() {
    timer_disable_intr(timer_group,0);
    esp_intr_free(timer_isr_handle);
    esp_intr_free(gpio_isr_handle);
    button_deinit();
}