/**
 * @file gpio.h
 * @author Octavio Perez Balcedo
 * @brief GPIO handling module for controlling specific output pins.
 *
 * This file provides functions to initialize and control GPIO pins, avoiding 
 * direct inclusion of platform-specific headers in sensor drivers, thus enhancing portability.
 *
 * @date 2025-02-09
 */

#ifndef GPIO_H
#define GPIO_H

#include "driver/gpio.h"
#include "esp_log.h"
#include <stdbool.h>
#include <stdint.h>

/**
 * @enum gpio_t
 * @brief Enumeration for GPIO pins used in the system.
 */
typedef enum {
    GPIO_XSHUT_FIRST = GPIO_NUM_23, 
    GPIO_XSHUT_SECOND = GPIO_NUM_1,
    GPIO_XSHUT_THIRD = GPIO_NUM_2
} gpio_t;

/**
 * @brief Initializes the configured GPIO pins as output.
 *
 * This function configures the GPIO pins as output and ensures they start 
 * at a low logic level.
 *
 * @return ESP_OK on success, ESP_FAIL on error.
 */
esp_err_t gpio_init(void);

/**
 * @brief Sets the output state of a specified GPIO pin.
 *
 * This function controls the logic level of a given GPIO pin.
 *
 * @param gpio The GPIO pin to modify.
 * @param enable True to set high (1), false to set low (0).
 * @return ESP_OK on success, ESP_FAIL if setting the level fails.
 */
esp_err_t gpio_set_output(gpio_t gpio, bool enable);

#endif /* GPIO_H */
