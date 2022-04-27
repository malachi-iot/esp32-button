#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "esp_log.h"

#include "button.h"
#include "internal/esp32-button.h"

// DEBT: Either static-ize this or prefix to avoid collisions
extern int pin_count;
extern debounce_t * debounce;
extern QueueHandle_t queue;

static gpio_isr_handle_t isr_handle;

static const char* TAG = "esp32 Button ISR";

static void update_button(debounce_t *d, uint32_t level) {
    d->history = (d->history << 1) | level;
}

static void send_event(debounce_t db, int ev) {
    button_event_t event = {
        .pin = db.pin,
        .event = ev,
    };
    xQueueSendToBackFromISR(queue, &event, NULL);
}

// FIX: Not valid for multiple buttons and not debouncing anything at all
// Just getting things compiling and running for now
static void button_isr(void* context) {
    // Guidance from:
    // https://esp32.com/viewtopic.php?t=345 
    // 
    uint32_t gpio_intr_status = READ_PERI_REG(GPIO_STATUS_REG);   //read status to get interrupt status for GPIO0-31
    uint32_t gpio_intr_status_h = READ_PERI_REG(GPIO_STATUS1_REG);//read status1 to get interrupt status for GPIO32-39
    // Fun fact - your ESP32 will reset if you don't clear your interrupts :)
    SET_PERI_REG_MASK(GPIO_STATUS_W1TC_REG, gpio_intr_status);    //Clear intr for gpio0-gpio31
    SET_PERI_REG_MASK(GPIO_STATUS1_W1TC_REG, gpio_intr_status_h); //Clear intr for gpio32-39

    //ets_printf("1 Intr GPIO%d ,val: %d\n",gpio_num,gpio_get_level(gpio_num));
    ets_printf("1 Intr\n");

    button_event_t event = {
        .pin = 0,
        .event = BUTTON_DOWN,
    };
    xQueueSendToBackFromISR(queue, &event, NULL);

    ets_printf("2 Intr: pin_count=%d\n", pin_count);

    for (int idx=0; idx<pin_count; idx++) {
        //update_button(&debounce[idx], gpio_intr_status);
        send_event(debounce[idx], BUTTON_DOWN);
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
        gpio_isr_register(button_isr, NULL, ESP_INTR_FLAG_LEVEL1, &isr_handle));

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

QueueHandle_t button_init_isr(unsigned long long pin_select) {
    QueueHandle_t queue = esp32_button_init_internal(pin_select, GPIO_FLOATING);
    return queue;
}

void button_deinit_isr() {
    esp_intr_free(isr_handle);
    button_deinit();
}