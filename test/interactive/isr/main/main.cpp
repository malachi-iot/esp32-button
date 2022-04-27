#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_chip_info.h"

#include "esp_log.h"

#include "button.h"

extern "C" void app_main(void)
{
    const char* TAG = "app_main";

    ESP_LOGI(TAG, "Interactive button test");

    /* Print chip information */
    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);
    ESP_LOGI(TAG, "This is %s chip with %d CPU core(s)",
            CONFIG_IDF_TARGET,
            chip_info.cores);

    QueueHandle_t button_events = button_init_isr(PIN_BIT(CONFIG_INTERACTIVE_BUTTON_GPIO));

    for(;;)
    {
        static int counter = 0;

        button_event_t ev;

        if(xQueueReceive(button_events, &ev, 5000 / portTICK_PERIOD_MS))
            ESP_LOGI(TAG, "ev.pin=%d, ev.event=%d", ev.pin, ev.event);
        else
            ESP_LOGD(TAG, "Counting = %d", ++counter);
    }
}