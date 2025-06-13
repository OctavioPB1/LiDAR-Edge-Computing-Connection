/**
 * @file http_handler.c
 * @author Guerrico Leonel (lguerrico99@gmail.com)
 * 
 * @brief Implementation of the module for retrieving instructions via HTTP.
 * 
 * This file implements the functionality to connect to an HTTP server, fetch 
 * instructions in JSON format, and execute corresponding actions based on the 
 * received data.
 * 
 * @note: This library is a patch for an error in the MQTT implementation  
 * that makes it impossible for us to receive instructions through it.  
 * It must be deprecated once it is fixed.  
 * 
 * @date 2025-02-09
 */

#include "http_handler.h"
#include "frozen_json_helper.h"
#include "esp_log.h"
#include "esp_http_client.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include "instruction_buffer.h"
#include "debug_helper.h"

#define INST_MAX_SIZE 20    // Maximum size of the instruction

static const char *TAG = "HTTP_HANDLER";
static const char *URL = "http://192.168.1.100:1880/instruction/last"; // Backend URL

// Static function declaration
static void decodeInstruction(int, char *);

/**
 * @brief HTTP client event handler, responsible for processing received data.
 *
 * This function is triggered when the HTTP client receives data. If the event is of type 
 * `HTTP_EVENT_ON_DATA`, the received data is extracted and passed to `decodeInstruction()`
 * for further processing.
 *
 * @param evt HTTP client event handle.
 * @return ESP_OK (Always returns OK, as errors are not handled at this level).
 */
esp_err_t client_event_get_handler(esp_http_client_event_handle_t evt)
{
    switch (evt->event_id)
    {
    case HTTP_EVENT_ON_DATA:
        // Executes when data is received from the server.
        decodeInstruction(evt->data_len, (char *)evt->data);
        break;

    default:
        break;
    }
    return ESP_OK;
}

/**
 * @brief Sends an HTTP GET request to fetch the latest instruction from the server.
 *
 * This function initializes an HTTP client and sends a GET request to the predefined URL.
 * If the request is successful, the received data is processed through the event handler.
 * In case of an error, it is logged accordingly.
 *
 * @return 
 * - ESP_OK: If the request was successful.
 * - ESP_FAIL: If there was an error initializing the client or in the HTTP request.
 */
esp_err_t getHTTPInstruction(void)
{
    esp_err_t err;

    // HTTP client configuration
    esp_http_client_config_t config = {
        .url = URL,
        .timeout_ms = 5000,
        .cert_pem = NULL,
        .event_handler = client_event_get_handler};

    // Initialize the HTTP client
    esp_http_client_handle_t client = esp_http_client_init(&config);
    if (!client)
    {
        DEBUGING_ESP_LOG(ESP_LOGE(TAG, "Failed to initialize HTTP client"));
        LOG_MESSAGE_E(TAG, "Failed to initialize HTTP client");
        return ESP_FAIL;
    }

    // Perform the GET request
    err = esp_http_client_perform(client);
    if (err != ESP_OK)
    {
        DEBUGING_ESP_LOG(ESP_LOGE(TAG, "ERROR PERFORMING GET %s",esp_err_to_name(err)));
        LOG_MESSAGE_E(TAG, "ERROR PERFORMING GET");

        esp_http_client_cleanup(client);
        vTaskDelay(100 / portTICK_PERIOD_MS);
        return ESP_FAIL;
    }

    // Cleanup and delay before finishing
    esp_http_client_cleanup(client);
    vTaskDelay(100 / portTICK_PERIOD_MS);
    return ESP_OK;
}

/**
 * @brief Decodes the received instruction in JSON format and executes the corresponding action.
 *
 * This function extracts the instruction from the JSON-formatted string and converts it 
 * into a readable command. If the command is "REBOOT", it triggers a microcontroller reset.
 * If the command is "ABORT", it logs the action without executing it. Otherwise, the 
 * instruction is stored in a buffer for further processing.
 *
 * @param length Length of the received instruction string.
 * @param str Instruction string in JSON format.
 */
static void decodeInstruction(int length, char *str)
{
    char msg[INST_MAX_SIZE] = "";
    esp_err_t err = deserialize_json_data(str, msg, INST_MAX_SIZE);
    if (err != ESP_OK)
    {
        DEBUGING_ESP_LOG(ESP_LOGE(TAG, "ERORR DECODING JSON"));
        LOG_MESSAGE_E(TAG, "ERORR DECODING JSON");

    }
    else
    {
        DEBUGING_ESP_LOG(ESP_LOGW(TAG, "INST: %s", msg));
        // Compare the received instruction with expected commands
        if (strncmp(msg, "REBOOT", strlen("REBOOT")) == 0)
        {
            DEBUGING_ESP_LOG(ESP_LOGW(TAG, "Reiniciando el MCU..."));
            LOG_MESSAGE_W(TAG, "Reiniciando el MCU...");
            esp_restart(); // Restart the microcontroller
        }
        else if (strncmp(msg, "ABORT", strlen("ABORT")) == 0)
        {
            DEBUGING_ESP_LOG(ESP_LOGW(TAG, "Abortando..."));
            LOG_MESSAGE_W(TAG, "Abortando...");
            //ABORT IS STILL IN DEVELOPMENT
        }
        else
        {
            // Save the instruction in a buffer for further processing
            saveInstruction(msg);
        }
    }
}