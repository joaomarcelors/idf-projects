#pragma once

#include <stdio.h>
#include "i2c_bus.h"

typedef void* mcp23017_handle_t;                        /*!< handle of mcp23017 */

typedef enum {
    MCP23017_GPIOA = 0x00, /*!< GPIO Port A */
    MCP23017_GPIOB,        /*!< GPIO Port B */
} mcp23017_gpio_port_t; /*!< GPIO Port Type */

typedef enum {
    MCP23017_GPIO_MODE_OUTPUT = 0x00,        /*!< GPIO Mode OUTPUT*/
    MCP23017_GPIO_MODE_INPUT = 0x01, /*!< GPIO Mode INPUT*/
    MCP23017_GPIO_MODE_INPUT_PULLUP = 0x02, /*!< GPIO Mode INPUT_PULLUP*/
} mcp23017_gpio_mode_t; /*!< GPIO Mode Type */

typedef enum {
    MCP23017_GPIO_LOW = 0x00, /*!< GPIO LOW State*/
    MCP23017_GPIO_HIGH = 0x01,        /*!< GPIO HiGH State*/
} mcp23017_gpio_state_t; /*!< GPIO State Type */

typedef enum {
    MCP23017_NOPIN = 0x0000,  /*!< GPIO Pin Num */
    MCP23017_PIN0 = 0x0001,  /*!< GPIO Pin Num */
    MCP23017_PIN1 = 0x0002,  /*!< GPIO Pin Num */
    MCP23017_PIN2 = 0x0004,  /*!< GPIO Pin Num */
    MCP23017_PIN3 = 0x0008,  /*!< GPIO Pin Num */
    MCP23017_PIN4 = 0x0010,  /*!< GPIO Pin Num */
    MCP23017_PIN5 = 0x0020,  /*!< GPIO Pin Num */
    MCP23017_PIN6 = 0x0040,  /*!< GPIO Pin Num */
    MCP23017_PIN7 = 0x0080,  /*!< GPIO Pin Num */
    MCP23017_PIN8 = 0x0100,  /*!< GPIO Pin Num */
    MCP23017_PIN9 = 0x0200,  /*!< GPIO Pin Num */
    MCP23017_PIN10 = 0x0400,  /*!< GPIO Pin Num */
    MCP23017_PIN11 = 0x0800,  /*!< GPIO Pin Num */
    MCP23017_PIN12 = 0x1000,  /*!< GPIO Pin Num */
    MCP23017_PIN13 = 0x2000,  /*!< GPIO Pin Num */
    MCP23017_PIN14 = 0x4000,  /*!< GPIO Pin Num */
    MCP23017_PIN15 = 0x8000,  /*!< GPIO Pin Num */
    MCP23017_ALLPINS = 0xFFFF,  /*!< GPIO Pin Num */
} mcp23017_pin_t; /*!< GPIO Pin Num Type, include all ports*/

static i2c_bus_handle_t i2c_bus = NULL;
static mcp23017_handle_t dev = NULL;

#ifdef __cplusplus
extern "C"
{
#endif

/**
 * @brief Check device Present
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 *     - ESP_ERR_INVALID_ARG Parameter error
 */
esp_err_t mcp23017_check_present();

/**
 * @brief Create a MCP23017 device
 *
 * @param bus device handle of i2c_bus
 * @param dev_addr device address
 *
 * @return
 *     - mcp23017_handle_t return mcp23017 device handle, NULL is failed.
 */
mcp23017_handle_t mcp23017_create(i2c_bus_handle_t bus, uint8_t dev_addr);

/**
 * @brief Delete the MCP23017 device
 *
 * @param p_dev pointer to the device handle of MCP23017
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 *     - ESP_ERR_INVALID_ARG Parameter error
 */
esp_err_t mcp23017_delete(mcp23017_handle_t *p_dev);

/**
 * @brief Deinit MCP23017
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t mcp23017_deinit();

/**
 * @brief read value of GPIO 
 *
 * @param gpio_num GPIO [0-15] of mcp23017 
 *
 * @return
 *     - uint8_t value of level
 */
uint8_t mcp23017_get_gpio_level(mcp23017_pin_t gpio_num);

/**
 * @brief get interrupt flag of GPIO
 *
 * @return
 *     - uint16_t value of GPIO interrupt flag
 */
uint16_t mcp23017_get_int_flag();

/**
 * @brief gets the interrupt capture values for pins with interrupts enabled,
 *          and gpio values for the ones that aren't.
 *          clear interrupt flag
 *
 * @return 
 *     - uint16_t value of interrupt pin
 */
uint16_t mcp23017_get_int_pin();

/**
 * @brief Init MCP23017
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 *     - ESP_ERR_INVALID_ARG Parameter error
 */
esp_err_t mcp23017_init();

/**
 * @brief Set MCP23017 interrupt pin,
 *          Only in GPIO work in INPUT mode,
 *              Default:compare last value,
 *
 * @param pins pin of interrupt
 * @param intr_mode 0: compared against previous, 1: compared against DEFVAL register.
 * @param defaultValue pins default level
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 *     - ESP_ERR_INVALID_ARG Parameter error
 */
esp_err_t mcp23017_interrupt_en(uint16_t pins, bool intr_mode, uint16_t defaultValue);

/**
 * @brief delete MCP23017 interrupt pin,
 *
 * @param pins pin of interrupt
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 *     - ESP_ERR_INVALID_ARG Parameter error
 */
esp_err_t mcp23017_interrupt_disable(uint16_t pins);

/**
 * @brief Set the polarity of the INT output pin
 *
 * @param gpio pin of interrupt
 * @param chLevel interrupt polarity
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 *     - ESP_ERR_INVALID_ARG Parameter error
 */
esp_err_t mcp23017_set_interrupt_polarity(mcp23017_gpio_port_t gpio, uint8_t chLevel);

/**
 * @brief Sequential operation mode set
 *
 * @param isSeque 1:Prohibit sequential operation, the address pointer is not incremented
 *                  0:Enable sequential operation, address pointer increment
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 *     - ESP_ERR_INVALID_ARG Parameter error
 */
esp_err_t mcp23017_set_seque_mode(uint8_t isSeque);

/**
 * @brief Set MCP23017 pin pullup
 *
 * @param pins pin of pullup
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 *     - ESP_ERR_INVALID_ARG Parameter error
 */
esp_err_t mcp23017_set_pullup(uint16_t pins);

/**
 * @brief Set MCP23017 GPIOA/GPIOB Mirror:1:Interrupt inconnect; 0:not connect.
 *
 * @param mirror whether set up mirror interrupt
 * @param gpio select GPIOA/GPIOB
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 *     - ESP_ERR_INVALID_ARG Parameter error
 */
esp_err_t mcp23017_mirror_interrupt(uint8_t mirror, mcp23017_gpio_port_t gpio);

/**
 * @brief read value of REG_GPIOA/REG_GPIOB;Reflects the logic level on pin <7: 0>
 *
 * @param gpio GPIO of mcp23017
 *
 * @return
 *     - uint8_t value of level
 */
uint8_t mcp23017_read_io(mcp23017_gpio_port_t gpio);

/**
 * @brief set Direction of all GPIO;
 *
 * @param mode Mode Input/Input_Pullup or Output 
 *
 * @return
 *     - NULL
 */
void mcp23017_set_all_gpio_dir(mcp23017_gpio_mode_t mode);

/**
 * @brief set level of GPIO;
 *
 * @param gpio_num GPIO [0-15] of mcp23017
 * @param level MCP23017_GPIO_LOW or MCP23017_GPIO_HIGH
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 *     - ESP_ERR_INVALID_ARG Parameter error
 */
esp_err_t mcp23017_set_gpio_level(mcp23017_pin_t gpio_num, mcp23017_gpio_state_t level);

/**
 * @brief set level of all GPIO;
 *
 * @param level MCP23017_GPIO_LOW or MCP23017_GPIO_HIGH
 *
 * @return
 *     - NULL
 */
void mcp23017_set_all_gpio_level(mcp23017_gpio_state_t level);

/**
 * @brief set Direction of individual GPIO;
 *
 * @param gpio_num GPIO [0-15] of mcp23017
 * @param mode Mode Input/Input_Pullup or Output 
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 *     - ESP_ERR_INVALID_ARG Parameter error
 */
esp_err_t mcp23017_set_gpio_dir(mcp23017_pin_t gpio_num, mcp23017_gpio_mode_t mode);

/**
 * @brief set Direction of GPIOA;Set the logic level on pin <7: 0>, 0 - output, 1 - input,
 *
 * @param value value of GPIOX
 * @param gpio GPIO of mcp23017
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 *     - ESP_ERR_INVALID_ARG Parameter error
 */
esp_err_t mcp23017_set_io_dir(uint8_t value, mcp23017_gpio_mode_t mode, mcp23017_gpio_port_t gpio);

/**
 * @brief write output value of GPIOA,(work in output)
 *
 * @param value value of GPIOX
 * @param gpio GPIO of mcp23017
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 *     - ESP_ERR_INVALID_ARG Parameter error
 */
esp_err_t mcp23017_write_io(uint8_t value, mcp23017_gpio_port_t gpio);

#ifdef __cplusplus
}
#endif