/**
 * @file limit_switch.c
 * @author Guerrico Leonel (lguerrico@outlook.com)
 * @brief Implementation of limit switch handling for ESP32.
 * 
 * This module sets up an interrupt-driven limit switch using GPIO.
 * When the switch is activated, an ISR triggers and a flag is set.
 * 
 * @version 1.0
 * @date 2024-12-05
 */

#include "limit_switch.h"
#include "esp_log.h"
#include "servo_compatibility.h"
#include "esp_timer.h"          // For getting the current time in microseconds
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "debug_helper.h"

/** @brief Logging tag for debugging */
const char *TAG = "Servo Interruptions";
/** @brief Flag to indicate if the limit switch was triggered */
static volatile bool limit_switch_triggered = false;
/** @brief Timestamp of the last activation, used for debounce */ 
static int64_t last_trigger_time = 0;
/** @brief Debounce delay in microseconds (50 ms) */                
const int64_t debounce_delay = 500000;               
/** @brief Semaphore for synchronizing limit switch events */
static SemaphoreHandle_t flag_semaphore;


static void limit_switch_isr_handler(void *);

/** 
 * @brief Interrupt Service Routine (ISR) for the limit switch.
 * 
 * This function is executed when the limit switch is triggered.
 * It implements a debounce mechanism to avoid false triggers.
 * 
 * @param arg Unused parameter (for ISR compatibility).
 */
static void limit_switch_isr_handler(void *arg)
{
    int64_t current_time = esp_timer_get_time();
    if ((current_time - last_trigger_time) > debounce_delay)
    {
        limit_switch_triggered = true;
        last_trigger_time = current_time; 
        xSemaphoreGiveFromISR(flag_semaphore, NULL);
    }
}

/**
 * @brief Initializes the limit switch GPIO and ISR.
 * 
 * Configures the limit switch GPIO pin as an input with a pull-up resistor.
 * Sets up an interrupt that triggers on a falling edge (high-to-low transition).
 * 
 * @return ESP_OK on success, ESP_FAIL on failure.
 */
esp_err_t interrupt_init()
{
    // Create and initialize the semaphore
    flag_semaphore = xSemaphoreCreateBinary();
    xSemaphoreGive(flag_semaphore);

    // Configure GPIO as input
    esp_err_t err = gpio_set_direction(LIMIT_SWITCH_PIN, GPIO_MODE_INPUT);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "GPIO error. Failed to set direction.");
        LOG_MESSAGE_E(TAG, "GPIO error. Failed to set direction.");
        return err;
    }

    // Set interrupt trigger type (falling edge)
    err = gpio_set_intr_type(LIMIT_SWITCH_PIN, GPIO_INTR_NEGEDGE); 
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Parameter error. Failed to set interrupt trigger type.");
        LOG_MESSAGE_E(TAG,"Parameter error. Failed to set interrupt trigger type.");
        return err;
    }

    // Enable pull-up resistor
    err = gpio_pullup_en(LIMIT_SWITCH_PIN);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Parameter error. Failed to enable pull-up on GPIO.");
        LOG_MESSAGE_E(TAG,"Parameter error. Failed to enable pull-up on GPIO.");
        return err;
    }

    // Install ISR service
    err = gpio_install_isr_service(0);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to install the GPIO driver's ISR handler service.");
        LOG_MESSAGE_E(TAG,"Failed to install the GPIO driver's ISR handler service.");
        return err;
    }

    // Attach ISR handler
    err = gpio_isr_handler_add(LIMIT_SWITCH_PIN, limit_switch_isr_handler, NULL);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to add ISR handler for the corresponding GPIO pin.");
        LOG_MESSAGE_E(TAG,"Failed to add ISR handler for the corresponding GPIO pin.");
        return err;
    }

    return ESP_OK;
}


/**
 * @brief Checks if the limit switch was triggered.
 * 
 * This function waits until the semaphore is available, then checks if the 
 * limit switch was triggered. If so, it logs a message and inverts the servo direction.
 */
void check_limit_switch()
{

    if (xSemaphoreTake(flag_semaphore, portMAX_DELAY) == pdTRUE)
    {
        if (limit_switch_triggered)
        {
            DEBUGING_ESP_LOG(ESP_LOGW(TAG, "Limit switch triggered! Servo has reached the target position."));
            //ESP_LOGE(TAG,"LIMIT SWITCH");
            servo_invert();
            limit_switch_triggered = false; 
            DEBUGING_ESP_LOG(ESP_LOGW(TAG, "#"));
        }
        xSemaphoreGive(flag_semaphore);
    }
}

/**
 * @brief Deletes the flag semaphore.
 * 
 * This function frees the semaphore used for handling limit switch events.
 * 
 * @return ESP_OK on success, ESP_FAIL if the semaphore was NULL.
 */
esp_err_t delete_flag_semaphore()
{
    if (flag_semaphore != NULL) {
        vSemaphoreDelete(flag_semaphore);
    }
    else {
        return ESP_FAIL;
    }
    return ESP_OK;
}