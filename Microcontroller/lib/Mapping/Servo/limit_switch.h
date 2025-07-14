/**
 * @file limit_switch.h
 * @author Guerrico Leonel (lguerrico@outlook.com)
 * @brief Limit switch handling for ESP32.
 * 
 * This library provides functions to initialize and handle a limit switch using GPIO interrupts.
 * The limit switch is used to detect mechanical limits and trigger an action when reached.
 *
 * @version 1.0
 * @date 2024-12-05
 */

#ifndef _LIMIT_SWITCH_H_
#define _LIMIT_SWITCH_H_

#include <driver/gpio.h>
#include "servo_compatibility.h"

/** 
 * @brief GPIO pin used for the limit switch.
 */
#define LIMIT_SWITCH_PIN GPIO_NUM_25 

/**
 * @brief Initializes the GPIO interrupt for the limit switch.
 * 
 * Configures the specified GPIO as an input with a pull-up resistor.
 * Enables an interrupt service routine (ISR) that triggers when the pin goes from high to low.
 * 
 * @return ESP_OK on success, ESP_FAIL on failure.
 */
esp_err_t interrupt_init(void);

/**
 * @brief Checks if the limit switch has been triggered.
 * 
 * This function waits until the semaphore is available and processes the 
 * limit switch event. If triggered, it inverts the servo direction and resets the state.
 */
void check_limit_switch(void);

/**
 * @brief Deletes the flag semaphore.
 * 
 * Cleans up resources by deleting the semaphore used for synchronizing the limit switch event.
 * 
 * @return ESP_OK on success, ESP_FAIL if the semaphore was NULL.
 */
esp_err_t delete_flag_semaphore(void);

#endif /* _LIMIT_SWITCH_H_ */