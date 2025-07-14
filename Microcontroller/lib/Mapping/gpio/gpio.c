/**
 * @file gpio.c
 * @author Octavio Perez Balcedo
 * @brief Implementation of GPIO handling functions for VL53L0X.
 *
 * This file contains the implementation of functions to configure and control 
 * GPIO output pins, ensuring proper initialization and control.
 *
 * @date 2025-02-09
 */

#include "gpio.h"
#include "esp_err.h"

static const char *TAG = "GPIO";

/**
 * @brief Initializes the configured GPIO pins as output.
 *
 * This function sets up the GPIO pins as output and initializes them to a low state.
 *
 * @return ESP_OK on success, ESP_FAIL on error.
 */
esp_err_t gpio_init(void)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << GPIO_XSHUT_FIRST),     /**< Pin bitmask for output configuration. */
        .mode = GPIO_MODE_OUTPUT,                       /**< Set mode to output. */
        .pull_up_en = GPIO_PULLUP_DISABLE,              /**< Disable internal pull-up resistor. */
        .pull_down_en = GPIO_PULLDOWN_DISABLE,          /**< Disable internal pull-down resistor. */
        .intr_type = GPIO_INTR_DISABLE                  /**< Disable interrupts. */
    };

    if (gpio_config(&io_conf) != ESP_OK) {
        ESP_LOGE(TAG, "Error en el inicio del GPIO: direction XSHUT_FIRST");
        return ESP_FAIL;
    }

    // Ensure the pin starts at a low level
    if (gpio_set_level(GPIO_XSHUT_FIRST, 0) != ESP_OK ) {
        ESP_LOGE(TAG, "Error en el inicio del GPIO: level XSHUT_FIRST");
        return ESP_FAIL;
    }

    return ESP_OK;
}

/**
 * @brief Sets the output state of a specified GPIO pin.
 *
 * This function modifies the logic level of a given GPIO pin.
 *
 * @param gpio The GPIO pin to modify.
 * @param enable True to set high (1), false to set low (0).
 * @return ESP_OK on success, ESP_FAIL if setting the level fails.
 */
esp_err_t gpio_set_output(gpio_t gpio, bool enable)
{
    if (gpio_set_level(gpio, enable ? 1 : 0) == ESP_OK) {
        return ESP_OK;
    }
    ESP_LOGE(TAG, "Error en el seteo del GPIO: fallo al setear el nivel de XSHUT_FIRST");
    return ESP_FAIL;
}
