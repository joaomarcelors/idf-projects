#include <stdio.h>
#include "esp_log.h"
#include "esp_timer.h"
#include "mcp23017_component.h"

#define GPIO_BUTTON 7 //A7
#define GPIO_LED 9   //B1

void app_main(void){

    //initializing mcp23017
    mcp23017_init();
    
    //setting the GPIO for input or output
    mcp23017_set_gpio_dir(GPIO_BUTTON, MCP23017_GPIO_MODE_INPUT_PULLUP);
    mcp23017_set_gpio_dir(GPIO_LED, MCP23017_GPIO_MODE_OUTPUT);

    uint64_t last_time = esp_timer_get_time();
    uint8_t led_state = 0;

    while (true) {
        
        //reding gpio button
        uint8_t read_gpio = mcp23017_get_gpio_level(GPIO_BUTTON);

        //showing status 
        ESP_LOGI(__func__, "BUTTON state: %d", read_gpio);

        //changing the LED state to on/off every 1s without using delay
        if ((esp_timer_get_time() - last_time ) > 500000){
            led_state = !led_state;
            mcp23017_set_gpio_level(GPIO_LED, led_state);
            ESP_LOGI(__func__, "LED: %s", (led_state == 1? "On" : "Off"));
            last_time = esp_timer_get_time();
        }

        vTaskDelay(50 / portTICK_RATE_MS);
        
    }

    //deinitting mcp23017
    mcp23017_deinit();
}