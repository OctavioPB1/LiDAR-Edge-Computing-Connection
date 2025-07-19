/**
 * @file mqtt_server.c
 * @author Guerrico Leonel (lguerrico@outlook.com)
 * @brief Implementation of the MQTT client for ESP-IDF
 *
 * This source file contains the implementation of MQTT client functionalities,
 * including connecting to an MQTT broker, publishing messages, subscribing to topics,
 * handling MQTT events, and managing received instructions.
 *
 * WARNING: The receive instructions function is not currently working, 
 * due to a conflict in the way we implemented it with FreeRTOS.
 * It should be checked and correct to MQTT be fully implemented. 
 * Meanwhile the http_handler library manages the receive instruction.
 *
 * @note Ensure the MQTT broker is reachable and configured properly in `URL`.
 *
 * @version 1.0
 * @date 2024-12-04
 */

#include "mqtt_server.h"
#include "esp_log.h"
#include "mqtt_client.h"
#include "instruction_buffer.h"
#include "esp_system.h"
#include "debug_helper.h"

// Constants and Global Variables
#define NUM_TOPICS 5 ///< Number of topics to subscribe to

static const char *TAG = "MQTT_SERVER";                                          ///< Log tag for MQTT Server
//static const char *URL = "mqtt://192.168.1.105:1883";                              ///< MQTT broker URL
static const char *URL = "mqtt://192.168.1.104:1883";  ///< IP del Nodo Edge     ///< MQTT broker URL   
static esp_mqtt_client_handle_t mqtt_client = NULL;                              ///< Handle for MQTT client
static const char *TOPICS[] = {"Instruction", "Messages", "Mapping", "Battery", "Barrier"}; ///< Topics to subscribe to
static char inst[40] = {0};                                                      ///< Buffer for instructions
static uint32_t MQTT_CONNEECTED = 0;                                             ///< MQTT connection status

// Function Prototypes
static esp_err_t mqtt_connect(void);
static esp_err_t mqtt_subscribe(const char *);
static void mqtt_event_handler(void *, esp_event_base_t, int32_t, void *);
static void mqtt_subscribing(void);
static void instruction_handler(char *, size_t length);

/**
 * @brief Initializes and starts the MQTT client
 *
 * This function attempts to connect the MQTT client to the broker. If the connection
 * fails, it retries up to `max_retries` before returning an error.
 *
 * @return
 * - ESP_OK on success
 * - ESP_FAIL on connection failure after retries
 */
esp_err_t mqtt_start()
{
    esp_err_t err = mqtt_connect();
    uint8_t retry_count = 0;
    const uint8_t max_retries = 5;
    while (err != ESP_OK && retry_count < max_retries)
    {
        DEBUGING_ESP_LOG(ESP_LOGW(TAG, "Reconnecting... attempt %d", retry_count + 1));
        vTaskDelay(2000 / portTICK_PERIOD_MS);
        err = mqtt_connect();
        retry_count++;
    }
    if (err != ESP_OK)
    {
        DEBUGING_ESP_LOG(ESP_LOGE(TAG, "Failed to connect to MQTT after %d retries", max_retries));
        return err;
    }

    return ESP_OK;
}

/**
 * @brief Establishes a connection with the MQTT broker
 *
 * Sets up the MQTT client configuration and starts the client. The connection status
 * is updated upon successful or failed attempts.
 *
 * @return
 * - ESP_OK on success
 * - ESP_FAIL on failure
 */
static esp_err_t mqtt_connect()
{
    DEBUGING_ESP_LOG(ESP_LOGI(TAG, "INITIALIZING MQTT"));
    const esp_mqtt_client_config_t mqtt_config = {
        .broker.address.uri = URL,

    };
    mqtt_client = esp_mqtt_client_init(&mqtt_config);
    if (mqtt_client == NULL)
    {
        DEBUGING_ESP_LOG(ESP_LOGE(TAG, "Failed to initialize MQTT client"));
        return ESP_FAIL;
    }

    esp_mqtt_client_register_event(mqtt_client, ESP_EVENT_ANY_ID, mqtt_event_handler, mqtt_client);

    esp_err_t err = esp_mqtt_client_start(mqtt_client);
    if (err != ESP_OK)
    {
        DEBUGING_ESP_LOG(ESP_LOGE(TAG, "Failed to start MQTT client: %s", esp_err_to_name(err)));
        return err;
    }

    DEBUGING_ESP_LOG(ESP_LOGI(TAG, "MQTT client connected to %s", URL));
    return ESP_OK;
}

/**
 * @brief Handles MQTT events
 *
 * Processes various MQTT events such as connection, disconnection, subscription,
 * message reception, and errors.
 *
 * @param[in] handler_args Unused
 * @param[in] base Event base
 * @param[in] event_id Event identifier
 * @param[in] event_data Pointer to event-specific data
 */
static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    DEBUGING_ESP_LOG(ESP_LOGD(TAG, "Event dispatched from event loop base=%s, event_id=%ld", base, event_id));
    esp_mqtt_event_handle_t event = event_data;
    switch ((esp_mqtt_event_id_t)event_id)
    {
    case MQTT_EVENT_CONNECTED:
        DEBUGING_ESP_LOG(ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED"));
        MQTT_CONNEECTED = 1;
        //mqtt_subscribing();
        break;
    case MQTT_EVENT_DISCONNECTED:
        DEBUGING_ESP_LOG(ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED"));
        MQTT_CONNEECTED = 0;
        break;

    case MQTT_EVENT_SUBSCRIBED:
        DEBUGING_ESP_LOG(ESP_LOGI(TAG, "MQTT_EVENT_SUBSCRIBED, msg_id=%d", event->msg_id));
        break;
    case MQTT_EVENT_UNSUBSCRIBED:
        DEBUGING_ESP_LOG(ESP_LOGI(TAG, "MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", event->msg_id));
        break;
    case MQTT_EVENT_PUBLISHED:
        DEBUGING_ESP_LOG(ESP_LOGI(TAG, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id));
        break;
    case MQTT_EVENT_DATA:
        DEBUGING_ESP_LOG(ESP_LOGI(TAG, "MQTT_EVENT_DATA - %s",event->topic));
        //Here should go the code to amnage the receive instruction
        /*if (strncmp(event->topic, "Instruction", event->topic_len) == 0)
        {
            ESP_LOGI(TAG, "Instruction received");

            instruction_handler(event->data, event->data_len);
        }
        else
        {
            ESP_LOGW(TAG, "ERROR: Message received from an unexpected Topic(%s)", event->topic);
        }*/
        break;
    case MQTT_EVENT_ERROR:
        DEBUGING_ESP_LOG(ESP_LOGI(TAG, "MQTT_EVENT_ERROR %s", event->data));
        break;
    default:
        DEBUGING_ESP_LOG(ESP_LOGI(TAG, "Other event id:%d", event->event_id));
        break;
    }
}

/**
 * @brief Subscribes to predefined MQTT topics
 *
 * Iterates through the `TOPICS` array and subscribes to each topic.
 */
static void mqtt_subscribing()
{
    esp_err_t err2;
    for (uint8_t i = 0; i < NUM_TOPICS; i++)
    {
        err2 = mqtt_subscribe(TOPICS[i]);
        if (err2 != ESP_OK)
        {
            DEBUGING_ESP_LOG(ESP_LOGW(TAG, "Error subscribing to topic: %s", TOPICS[i]));
        }
        else
        {
            DEBUGING_ESP_LOG(ESP_LOGI(TAG, "Successful subscription to: %s", TOPICS[i]));
        }
    }
}

/**
 * @brief Publishes a message to an MQTT topic.
 *
 * This function sends a message to the specified topic using the MQTT client.
 * The message has a default Quality of Service (QoS) of 1 and is not set as retained.
 *
 * @param[in] topic The topic to which the message should be published.
 * @param[in] payload The message to publish.
 *
 * @return
 *      - ESP_OK: If the message was published successfully.
 *      - ESP_FAIL: If the MQTT client is not initialized or an error occurs during publishing.
 */
esp_err_t mqtt_publish(const char *topic, const char *payload)
{
    if (mqtt_client == NULL)
    {
        DEBUGING_ESP_LOG(ESP_LOGE(TAG, "MQTT client not initialized"));
        return ESP_FAIL;
    }

    int msg_id = esp_mqtt_client_publish(mqtt_client, topic, payload, 0, 1, 0);
    if (msg_id == -1)
    {
        DEBUGING_ESP_LOG(ESP_LOGE(TAG, "Failed to publish message to topic %s", topic));
        return ESP_FAIL;
    }

    DEBUGING_ESP_LOG(ESP_LOGI(TAG, "Published message to topic %s", topic));
    return ESP_OK;
}

/**
 * @brief Subscribes to a specific MQTT topic.
 *
 * This function sends a request to subscribe to the specified MQTT topic.
 * If the MQTT client is not initialized, the subscription fails.
 *
 * @param[in] topic The topic to which you want to subscribe.
 *
 * @return
 *      - ESP_OK: If the subscription was successful.
 *      - ESP_FAIL: If the MQTT client is not initialized or the subscription fails.
 */
static esp_err_t mqtt_subscribe(const char *topic)
{
    if (mqtt_client == NULL)
    {
        DEBUGING_ESP_LOG(ESP_LOGE(TAG, "MQTT client not initialized"));
        return ESP_FAIL;
    }

    int msg_id = esp_mqtt_client_subscribe(mqtt_client, topic, 0);
    if (msg_id == -1)
    {
        DEBUGING_ESP_LOG(ESP_LOGE(TAG, "Failed to subscribe to topic %s", topic));
        return ESP_FAIL;
    }

    DEBUGING_ESP_LOG(ESP_LOGI(TAG, "Subscribed to topic %s", topic));
    return ESP_OK;
}

/**
 * @brief Disconnects the MQTT client
 *
 * Stops the MQTT client and releases resources. Resets the client handle to `NULL`.
 *
 * @return
 * - ESP_OK on success
 * - ESP_FAIL on failure
 */
esp_err_t mqtt_disconnect()
{
    if (mqtt_client == NULL)
    {
        DEBUGING_ESP_LOG(ESP_LOGE(TAG, "MQTT client not initialized"));
        return ESP_FAIL;
    }

    esp_err_t err = esp_mqtt_client_stop(mqtt_client);
    if (err != ESP_OK)
    {
        DEBUGING_ESP_LOG(ESP_LOGE(TAG, "Failed to stop MQTT client: %s", esp_err_to_name(err)));
        return err;
    }

    mqtt_client = NULL; // Limpiar el puntero del cliente
    DEBUGING_ESP_LOG(ESP_LOGI(TAG, "MQTT client disconnected"));
    return ESP_OK;
}

/**
 * @brief Handles received instruction messages
 *
 * Processes messages received on the "Instruction" topic, deserializing JSON data and
 * executing actions based on the instruction content (e.g., restarting the MCU).
 *
 * WARNING: This function is currently useless. Its causes a conflict with FreeRTOS, 
 * making the program reboot itself. It should be replaced with another one, to avoid conflict with FreeRTOS.
 * 
 * @param[in] data Pointer to the received message
 * @param[in] length Length of the message
 */
/*static void instruction_handler(char *str, size_t length)
{
    ESP_LOGW(TAG, "DATA RECEIVE %s", str);
    memset(inst, 0, sizeof(inst));
    char *message = (char *)malloc(length + 1);
    if (message == NULL)
    {
        ESP_LOGE(TAG, "Failed to allocate memory for MQTT message");
    }

    // Copiar los datos y agregar el terminador nulo manualmente
    memcpy(message, str, length);
    message[length] = '\0'; // Termina la cadena con '\0'
    esp_err_t err = deserealize_json_data(message, inst, length);
    free(message);
    ESP_LOGW(TAG, "Des Receive: %s", inst);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Error deserializing Instruction");
    }
    else
    {

        if (strncmp(inst, "REBOOT", strlen("REBOOT")) == 0)
        {
            ESP_LOGW("RESTART", "Reiniciando el MCU...");
            esp_restart(); // Llamada para reiniciar el MCU
        }
        else
        {
            ESP_LOGW(TAG, "Saving...");
            if (saveInstruction(inst) != ESP_OK)
            {
                ESP_LOGE(TAG, "Error saving Instruction");
            }
            ESP_LOGW(TAG, "SAVED");
        }
    }
}*/