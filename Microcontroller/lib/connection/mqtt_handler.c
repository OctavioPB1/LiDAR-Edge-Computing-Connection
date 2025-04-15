/**
 * @file mqtt_handler.c
 * @author
 *      Guerrico Leonel (lguerrico@outlook.com)
 * @brief Implementation of MQTT message handling for the system
 *
 * This file provides the core functionality for interacting with an MQTT broker.
 * It includes functions to send control messages (informational, warning, and error),
 * mapping data (distance and angle), and any other MQTT-related operations.
 *
 * The functions implemented here are designed for:
 * - Sending JSON-encoded messages via MQTT.
 * - Handling control messages and mapping data.
 * - Interfacing with other modules through structured JSON payloads.
 *
 * @version 1.0
 * @date 2024-12-04
 *
 */

#include "mqtt_handler.h"
#include "mqtt_server.h"
#include "esp_log.h"
#include "instruction_buffer.h"
#include "frozen_json_helper.h"
#include "debug_helper.h"

// Definitions
#define INSTRUCTIONS_BUFFER_SIZE 10       // Maximum number of instructions to store in buffer
#define INSTRUCTION_MESSAGE "Instruction" // Instruction Topic
#define CONTROL_MESSAGE "Messages"        // Message Topic
#define MAPPING_VALUE "Mapping"           // Mapping Value Topic
#define BATTERY_VALUE "Battery"           // Battery Level Topic

// Const
static const char *TAG = "MQTT_HANDLER"; // Library Tag

// Function Prototypes
static esp_err_t sendControlMessage(const char *, char *, const char *);

/**
 * @brief Get the Instruction Message
 *
 * This function get instruction from instruction buffer in instruction_buffer library.
 *
 * @param[out] msg Instruction
 * @return
 *      - ESP_OK on success
 *      - ESP_FAIL on fail
 */
esp_err_t getInstructionMessage(char *msg)
{

    return getInstruction(msg);
}

/**
 * @brief Sends a control message as a JSON payload.
 *
 * This funcion generate a JSON object from the parameters provided and send
 * it as a control message using the MQTT protocol.a64l
 *
 * The JSON structur is as follows:
 * {
 *  "tag": "<ESP_TAG>",
 *  "type": "<msg_type>",
 *  "message": "<msg>"
 *
 * }
 *
 * @param[in] ESP_TAG A label identifying the source library or component
 *                generating the message (e.g., "MQTT_HANDLER").
 * @param[in] msg_type Label that identify the severity of MessagePossible values are:
 *      - "Info": Informational messages.
 *      - "Warning": Warning messages.
 *      - "Error": Error messages.
 * @param[in] msg Message to send
 * @return
 *      - ESP_OK: If the message was successfully created and sent.
 *      - ESP_FAIL: If there was an error creating the JSON or sending the message.
 *
 * @note This function relies on the following functions:
 *      - `create_json_data`: Creates the JSON structure.
 *      - `print_json_data`: Logs the JSON for debugging.
 *      - `mqtt_publish`: Publishes the message via MQTT.
 *
 * @warning Ensure that `ESP_TAG`, `msg_type`, and `msg` are valid non-null
 *          strings to avoid undefined behavior.
 *
 */
static esp_err_t sendControlMessage(const char *ESP_TAG, char *msg_type, const char *msg)
{
    const char *values[3];
    char TAG_buffer[20], type_buffer[8], msg_buffer[50];
    snprintf(TAG_buffer, sizeof(TAG_buffer), "%s", ESP_TAG);
    snprintf(type_buffer, sizeof(type_buffer), "%s", msg_type);
    snprintf(msg_buffer, sizeof(msg_buffer), "%s", msg);
    values[0] = TAG_buffer;
    values[1] = type_buffer;
    values[2] = msg_buffer;
    const char *key[3] = {"tag", "type", "message"};
    char *json = NULL;

    esp_err_t err = create_json_data(&json, key, values, 3);

    if (err != ESP_OK)
    {
        if(json) free(json);
        
        DEBUGING_ESP_LOG(ESP_LOGE(TAG, "Error creating json data: %s", esp_err_to_name(err)));
        return err;
    }
    else
    {
        print_json_data(json);
    }

    err = mqtt_publish(CONTROL_MESSAGE, json);
    if (err != ESP_OK)
    {
        DEBUGING_ESP_LOG(ESP_LOGE(TAG, "Error publishing Control Message: %s", esp_err_to_name(err)));
        free(json);
        return err;
    }

    free(json);

    return ESP_OK;
}

/**
 * @brief Sends an informational control message.
 *
 * This function sends a control message with the severity level set to "INFO".
 * It wraps the `sendControlMessage` function for convenience.
 *
 * @param[in] TAG A label identifying the source library or component generating
 *            the message (e.g., "MQTT_HANDLER").
 * @param[in] msg The informational message content to send.
 *
 * @return
 *      - ESP_OK: If the message was successfully sent.
 *      - ESP_FAIL: If there was an error sending the message.
 *
 * @see sendControlMessage
 */
esp_err_t sendInfoMessage(const char *TAG, const char *msg)
{
    return sendControlMessage(TAG, "INFO", msg);
}

/**
 * @brief Sends a warning control message.
 *
 * This function sends a control message with the severity level set to "WARNING".
 * It wraps the `sendControlMessage` function for convenience.
 *
 * @param[in] TAG A label identifying the source library or component generating
 *            the message (e.g., "MQTT_HANDLER").
 * @param[in] msg The warning message content to send.
 *
 * @return
 *      - ESP_OK: If the message was successfully sent.
 *      - ESP_FAIL: If there was an error sending the message.
 *
 * @see sendControlMessage
 */

esp_err_t sendWarningMessage(const char *TAG, const char *msg)
{
    return sendControlMessage(TAG, "WARNING", msg);
}

/**
 * @brief Sends an error control message.
 *
 * This function sends a control message with the severity level set to "ERROR".
 * It wraps the `sendControlMessage` function for convenience.
 *
 * @param[in] TAG A label identifying the source library or component generating
 *            the message (e.g., "MQTT_HANDLER").
 * @param[in] msg The error message content to send.
 *
 * @return
 *      - ESP_OK: If the message was successfully sent.
 *      - ESP_FAIL: If there was an error sending the message.
 *
 * @see sendControlMessage
 */

esp_err_t sendErrorMessage( const char *TAG, const char *msg)
{
    return sendControlMessage(TAG, "ERROR", msg);
}

/**
 * @brief Sends mapping data as a JSON payload.
 *
 * This function generates a JSON object containing the mapping data, specifically
 * the distance and angle, and sends it using the MQTT protocol.
 *
 * The JSON structure is as follows:
 * {
 *   "distance": "<distance>",
 *   "angle": "<angle>"
 * }
 *
 * @param[in] distance The distance value to include in the mapping data.
 * @param[in] angle The angle value to include in the mapping data.
 *
 * @return
 *      - ESP_OK: If the mapping data was successfully sent.
 *      - ESP_FAIL: If there was an error creating or sending the JSON payload.
 *
 * @note This function relies on the following:
 *      - `create_json_data`: Creates the JSON structure.
 *      - `print_json_data`: Logs the JSON for debugging.
 *      - `mqtt_publish`: Publishes the JSON payload via MQTT.
 *
 * @warning Ensure the `distance` and `angle` parameters are valid values.
 *          The function converts them to strings with a maximum buffer size
 *          of 6 characters.
 */

esp_err_t sendMappingValue(uint16_t distance, int16_t angle)
{
    const char *values[2];
    char buffer1[6], buffer2[7];
    snprintf(buffer1, sizeof(buffer1), "%d", distance);
    snprintf(buffer2, sizeof(buffer2), "%i", angle);
    values[0] = buffer1;
    values[1] = buffer2;
    const char *keys[2] = {"distance", "angle"};
    char *json = NULL;

    esp_err_t err = create_json_data(&json, keys, values, 2);

    if (err != ESP_OK)
    {
        DEBUGING_ESP_LOG(ESP_LOGE(TAG, "Error creating json data: %s", esp_err_to_name(err)));
        return err;
    }
    else 
    {
        print_json_data(json);
    }

    err = mqtt_publish(MAPPING_VALUE, json);
    if (err != ESP_OK)
    {
        DEBUGING_ESP_LOG(ESP_LOGE(TAG, "Error publishing Control Message: %s", esp_err_to_name(err)));
        free(json);
        return err;
    }

    free(json);

    return ESP_OK;
}

esp_err_t sendBatteryLevel(uint8_t batteryLevel)
{
    const char *values[1];                                
    char buffer[6];                                       
    snprintf(buffer, sizeof(buffer), "%d", batteryLevel); 
    values[0] = buffer;                                   

    const char *keys[1] = {"level"};
    char *json = NULL;

    
    esp_err_t err = create_json_data(&json, keys, values, 1);
    print_json_data(json); 

    if (err != ESP_OK)
    {
        DEBUGING_ESP_LOG(ESP_LOGE(TAG, "Error creating json data: %s", esp_err_to_name(err)));
        free(json);
        return err;
    }

    err = mqtt_publish(BATTERY_VALUE, json);
    if (err != ESP_OK)
    {
        DEBUGING_ESP_LOG(ESP_LOGE(TAG, "Error publishing control message: %s", esp_err_to_name(err)));
        free(json);
        return err;
    }

    DEBUGING_ESP_LOG(ESP_LOGI(TAG, "Battery level published"));

    free(json);

    return ESP_OK;
}

