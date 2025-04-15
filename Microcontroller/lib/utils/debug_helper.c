/**
 * @file debug_helper.c
 * @author  Ossola Florencia (flor.ossola13@gmail.com)
 * 
 * @brief Implementation of logging helper for ESP32.
 * 
 * This file implements the logging function that sends messages to different 
 * handlers based on the log level. If an error occurs while sending, it logs 
 * the issue using ESP-IDF's logging system.
 * 
 * @date 2025-02-09
 *  
 */
 
#include "debug_helper.h"
#include "esp_log.h"
#include "mqtt_handler.h"

/**
 * @brief Sends a log message to the appropriate logging system.
 * 
 * This function determines the type of message (error, warning, or info) 
 * and calls the corresponding handler. If sending fails, an error is logged 
 * using ESP-IDF's logging system.
 * 
 * @param[in] level Log level (LOG_ERROR, LOG_WARNING, LOG_INFO).
 * @param[in] TAG   Identifier tag for the log message.
 * @param[in] fmt   Formatted log message string.
 */
void LOG_MESSAGE(int level, const char *TAG, char *fmt) {

    esp_err_t err = ESP_OK;

    // Send message based on log level
    switch(level){
        case LOG_ERROR:
            err = sendErrorMessage(TAG, fmt);
            break;
        case LOG_WARNING:
            err = sendWarningMessage(TAG, fmt);
            break;
        case LOG_INFO:
            err = sendInfoMessage(TAG, fmt);
            break;
    }

    // Log error if message sending fails
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error enviando mensaje de log: %s", fmt);
    }
}
