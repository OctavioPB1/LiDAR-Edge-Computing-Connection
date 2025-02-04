#ifndef GPIO_H
#define GPIO_H

#include "driver/gpio.h"
#include "esp_log.h"
#include <stdbool.h>
#include <stdint.h>

/**
 * Separate file for GPIO handling to avoid including MSP430.h directly
 * in the sensor drivers to make them more portable.
 */

typedef enum {
    GPIO_XSHUT_FIRST = GPIO_NUM_23,  // Cambia estos pines según tu hardware
    GPIO_XSHUT_SECOND = GPIO_NUM_1,
    GPIO_XSHUT_THIRD = GPIO_NUM_2
} gpio_t;

esp_err_t gpio_init(void);
esp_err_t gpio_set_output(gpio_t gpio, bool enable);

#endif /* GPIO_H */
