/*
 * SPDX-FileCopyrightText: 2022-2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "mcp23017.h"

void app_main(void){

    mcp23017_init();
    
    // TEST_ASSERT(ESP_OK == mcp23017_set_io_dir(device, 0x00, MCP23017_GPIOA)); //A:OUTPUT, B:INPUT
    // TEST_ASSERT(ESP_OK == mcp23017_set_io_dir(device, 0xff, MCP23017_GPIOB));

   // mcp23017_set_io_dir(0x00, MCP23017_GPIOB);
    mcp23017_set_all_gpio_dir(MCP23017_GPIO_MODE_INPUT_PULLUP);
    //mcp23017_set_pullup(0xf);
    // mcp23017_set_gpio_dir(0, MCP23017_GPIO_MODE_OUTPUT);
    // mcp23017_set_gpio_dir(1, MCP23017_GPIO_MODE_OUTPUT);
    // mcp23017_set_gpio_dir(8, MCP23017_GPIO_MODE_OUTPUT);
    // mcp23017_set_gpio_dir(9, MCP23017_GPIO_MODE_OUTPUT);

    // mcp23017_set_gpio_dir(15, MCP23017_GPIO_MODE_OUTPUT);
    //mcp23017_set_gpio_dir(15, MCP23017_GPIO_MODE_INPUT);
    //mcp23017_set_gpio_dir(9, MCP23017_GPIO_MODE_INPUT);

    //mcp23017_set_io_dir(device, 0xff, MCP23017_GPIOB);
    //mcp23017_set_io_dir(0x00, MCP23017_GPIOA);

    while (true) {
        
        /*****Interrupt Test******/
        // mcp23017_interrupt_en(device, MCP23017_PIN15, 1, 0x0000); //1: compared against DEFVAL register.
        // vTaskDelay(1000 / portTICK_RATE_MS);
        // printf("Intf:%d\n",  mcp23017_get_int_flag(device));

        /*****Normal Test*****/
        // TEST_ASSERT(ESP_OK == mcp23017_write_io(device, cnt, MCP23017_GPIOA));
        //uint8_t value = 0b0000010;
        //mcp23017_set_all_gpio_level(1);
        //mcp23017_write_io(value, MCP23017_GPIOB); 
        // mcp23017_set_gpio_level(0, MCP23017_GPIO_HIGH);
        // mcp23017_set_gpio_level(1, MCP23017_GPIO_HIGH);
        // mcp23017_set_gpio_level(9, MCP23017_GPIO_HIGH);

        // ESP_LOGE(TAG, "gpio 0: %d", mcp23017_get_gpio_level(0));
        // ESP_LOGE(TAG, "gpio 1: %d", mcp23017_get_gpio_level(1));
        // ESP_LOGE(TAG, "gpio 8: %d", mcp23017_get_gpio_level(8));
        // ESP_LOGE(TAG, "gpio 9: %d", mcp23017_get_gpio_level(9));
        
       //uint8_t readed = mcp23017_read_io(MCP23017_GPIOA);
        // if(readed & 0b0000010){
        //     printf("ligado\n");
        // }else{
        //     printf("desligado\n");
        // }
        //printf("GPIOB read = :%x\n", readed);
        // // //TEST_ASSERT_EQUAL_UINT8(cnt, readed);
        //mcp23017_set_all_gpio_level(0);
        //vTaskDelay(500 / portTICK_RATE_MS);
        //value = 0b0000000;
        //mcp23017_write_io(value, MCP23017_GPIOB); 
        //mcp23017_set_gpio_level(9, MCP23017_GPIO_LOW);
        // mcp23017_set_gpio_level(0, MCP23017_GPIO_LOW);
        // mcp23017_set_gpio_level(1, MCP23017_GPIO_LOW);
        //readed = mcp23017_read_io(MCP23017_GPIOB);
        // if(readed & 0b0000010){
        //     printf("ligado\n");
        // }else{
        //     printf("desligado\n");
        // }
        //printf("GPIOB read = :%x\n", readed);

        for(int i=0; i<16; i++){
            printf("%d", !mcp23017_get_gpio_level(i));
            if(i==7){
                printf(" ");
            }
        }
        printf("\n");

        vTaskDelay(50 / portTICK_RATE_MS);
        
    }

    mcp23017_deinit();
}

// static void mcp23017_test_read_write()
// {
//     uint8_t cnt = 5;

//     // TEST_ASSERT(ESP_OK == mcp23017_set_io_dir(device, 0x00, MCP23017_GPIOA)); //A:OUTPUT, B:INPUT
//     // TEST_ASSERT(ESP_OK == mcp23017_set_io_dir(device, 0xff, MCP23017_GPIOB));

//     mcp23017_set_io_dir(device, 0x00, MCP23017_GPIOA);
//     mcp23017_set_io_dir(device, 0xff, MCP23017_GPIOB);
    
//     while (cnt--) {

//         /*****Interrupt Test******/
//         // mcp23017_interrupt_en(device, MCP23017_PIN15, 1, 0x0000); //1: compared against DEFVAL register.
//         // vTaskDelay(1000 / portTICK_RATE_MS);
//         // printf("Intf:%d\n",  mcp23017_get_int_flag(device));

//         /*****Normal Test*****/
//         // TEST_ASSERT(ESP_OK == mcp23017_write_io(device, cnt, MCP23017_GPIOA));
//         mcp23017_write_io(device, cnt, MCP23017_GPIOA);
//         printf("GPIOA write = :%x\n", cnt);
//         uint8_t readed = mcp23017_read_io(device, MCP23017_GPIOB);
//         printf("GPIOB read = :%x\n", readed);
//         //TEST_ASSERT_EQUAL_UINT8(cnt, readed);
//         vTaskDelay(300 / portTICK_RATE_MS);
//     }
// }

// TEST_CASE("Device mcp23017 test, connect A-B port together", "[mcp23017][iot][device]")
// {
//     mcp23017_test_init();
//     mcp23017_test_read_write();
//     mcp23017_test_deinit();
// }
