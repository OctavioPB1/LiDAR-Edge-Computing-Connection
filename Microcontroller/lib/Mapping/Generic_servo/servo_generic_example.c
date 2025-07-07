/**
 * @file servo_generic_example.c
 * @brief Example usage of the generic servo motor control library
 * 
 * This file demonstrates how to use the generic servo library to control
 * different types of servo motors including position servos and continuous
 * rotation servos.
 * 
 * @version 1.0
 * @date 2025-01-27
 * @author Assistant
 */

#include "servo_generic.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "SERVO_EXAMPLE";

// Example task for position servo
void servo_position_example_task(void *pvParameters)
{
    servo_handle_t *servo = NULL;
    
    // Use predefined configuration for standard position servo
    esp_err_t ret = servo_generic_init(&SERVO_CONFIG_POSITION_STANDARD, &servo);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize position servo");
        vTaskDelete(NULL);
        return;
    }
    
    // Enable the servo
    ret = servo_generic_enable(servo);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enable position servo");
        servo_generic_deinit(servo);
        vTaskDelete(NULL);
        return;
    }
    
    ESP_LOGI(TAG, "Position servo example started");
    
    while (1) {
        // Move to 0 degrees
        ESP_LOGI(TAG, "Moving to 0 degrees");
        servo_generic_set_position(servo, 0);
        vTaskDelay(pdMS_TO_TICKS(2000));
        
        // Move to 90 degrees
        ESP_LOGI(TAG, "Moving to 90 degrees");
        servo_generic_set_position(servo, 90);
        vTaskDelay(pdMS_TO_TICKS(2000));
        
        // Move to 180 degrees
        ESP_LOGI(TAG, "Moving to 180 degrees");
        servo_generic_set_position(servo, 180);
        vTaskDelay(pdMS_TO_TICKS(2000));
        
        // Smooth movement from 180 to 0 degrees over 3 seconds
        ESP_LOGI(TAG, "Smooth movement from 180 to 0 degrees");
        servo_generic_move_smooth(servo, 0, 3000);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

// Example task for continuous rotation servo
void servo_continuous_example_task(void *pvParameters)
{
    servo_handle_t *servo = NULL;
    
    // Use predefined configuration for continuous rotation servo
    esp_err_t ret = servo_generic_init(&SERVO_CONFIG_CONTINUOUS_STANDARD, &servo);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize continuous servo");
        vTaskDelete(NULL);
        return;
    }
    
    // Enable the servo
    ret = servo_generic_enable(servo);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enable continuous servo");
        servo_generic_deinit(servo);
        vTaskDelete(NULL);
        return;
    }
    
    ESP_LOGI(TAG, "Continuous servo example started");
    
    while (1) {
        // Rotate clockwise at 50% speed
        ESP_LOGI(TAG, "Rotating clockwise at 50%% speed");
        servo_generic_set_speed(servo, 50, SERVO_DIR_CW);
        vTaskDelay(pdMS_TO_TICKS(3000));
        
        // Stop
        ESP_LOGI(TAG, "Stopping");
        servo_generic_stop(servo);
        vTaskDelay(pdMS_TO_TICKS(1000));
        
        // Rotate counter-clockwise at 75% speed
        ESP_LOGI(TAG, "Rotating counter-clockwise at 75%% speed");
        servo_generic_set_speed(servo, 75, SERVO_DIR_CCW);
        vTaskDelay(pdMS_TO_TICKS(3000));
        
        // Stop
        ESP_LOGI(TAG, "Stopping");
        servo_generic_stop(servo);
        vTaskDelay(pdMS_TO_TICKS(1000));
        
        // Variable speed demonstration
        for (int speed = 10; speed <= 100; speed += 10) {
            ESP_LOGI(TAG, "Rotating clockwise at %d%% speed", speed);
            servo_generic_set_speed(servo, speed, SERVO_DIR_CW);
            vTaskDelay(pdMS_TO_TICKS(500));
        }
        
        servo_generic_stop(servo);
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

// Example task for custom servo configuration
void servo_custom_example_task(void *pvParameters)
{
    servo_handle_t *servo = NULL;
    
    // Create custom configuration for a specific servo
    servo_config_t custom_config = servo_generic_create_config(
        GPIO_NUM_14,           // GPIO pin
        SERVO_TYPE_POSITION,   // Position servo
        600,                   // Min pulse (600µs)
        2400,                  // Max pulse (2400µs)
        1500,                  // Center pulse (1500µs)
        50,                    // 50Hz frequency
        0,                     // Min angle
        270                    // Max angle (270° servo)
    );
    
    esp_err_t ret = servo_generic_init(&custom_config, &servo);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize custom servo");
        vTaskDelete(NULL);
        return;
    }
    
    // Enable the servo
    ret = servo_generic_enable(servo);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enable custom servo");
        servo_generic_deinit(servo);
        vTaskDelete(NULL);
        return;
    }
    
    ESP_LOGI(TAG, "Custom servo example started (270° range)");
    
    while (1) {
        // Move to different positions in the 270° range
        ESP_LOGI(TAG, "Moving to 0 degrees");
        servo_generic_set_position(servo, 0);
        vTaskDelay(pdMS_TO_TICKS(2000));
        
        ESP_LOGI(TAG, "Moving to 90 degrees");
        servo_generic_set_position(servo, 90);
        vTaskDelay(pdMS_TO_TICKS(2000));
        
        ESP_LOGI(TAG, "Moving to 180 degrees");
        servo_generic_set_position(servo, 180);
        vTaskDelay(pdMS_TO_TICKS(2000));
        
        ESP_LOGI(TAG, "Moving to 270 degrees");
        servo_generic_set_position(servo, 270);
        vTaskDelay(pdMS_TO_TICKS(2000));
        
        // Smooth movement back to center
        ESP_LOGI(TAG, "Smooth movement to center (135 degrees)");
        servo_generic_move_smooth(servo, 135, 2000);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

// Example task demonstrating multiple servos
void servo_multiple_example_task(void *pvParameters)
{
    servo_handle_t *position_servo = NULL;
    servo_handle_t *continuous_servo = NULL;
    
    // Initialize position servo on GPIO 14
    servo_config_t pos_config = SERVO_CONFIG_POSITION_STANDARD;
    pos_config.pin = GPIO_NUM_14;
    
    esp_err_t ret = servo_generic_init(&pos_config, &position_servo);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize position servo");
        vTaskDelete(NULL);
        return;
    }
    
    // Initialize continuous servo on GPIO 15
    servo_config_t cont_config = SERVO_CONFIG_CONTINUOUS_STANDARD;
    cont_config.pin = GPIO_NUM_15;
    
    ret = servo_generic_init(&cont_config, &continuous_servo);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize continuous servo");
        servo_generic_deinit(position_servo);
        vTaskDelete(NULL);
        return;
    }
    
    // Enable both servos
    servo_generic_enable(position_servo);
    servo_generic_enable(continuous_servo);
    
    ESP_LOGI(TAG, "Multiple servo example started");
    
    while (1) {
        // Position servo to 0°, continuous servo clockwise
        ESP_LOGI(TAG, "Position: 0°, Continuous: CW");
        servo_generic_set_position(position_servo, 0);
        servo_generic_set_speed(continuous_servo, 50, SERVO_DIR_CW);
        vTaskDelay(pdMS_TO_TICKS(2000));
        
        // Position servo to 90°, continuous servo counter-clockwise
        ESP_LOGI(TAG, "Position: 90°, Continuous: CCW");
        servo_generic_set_position(position_servo, 90);
        servo_generic_set_speed(continuous_servo, 50, SERVO_DIR_CCW);
        vTaskDelay(pdMS_TO_TICKS(2000));
        
        // Position servo to 180°, stop continuous servo
        ESP_LOGI(TAG, "Position: 180°, Continuous: Stop");
        servo_generic_set_position(position_servo, 180);
        servo_generic_stop(continuous_servo);
        vTaskDelay(pdMS_TO_TICKS(2000));
        
        // Smooth movement and variable speed
        ESP_LOGI(TAG, "Smooth movement and variable speed");
        servo_generic_move_smooth(position_servo, 0, 3000);
        
        for (int speed = 10; speed <= 100; speed += 10) {
            servo_generic_set_speed(continuous_servo, speed, SERVO_DIR_CW);
            vTaskDelay(pdMS_TO_TICKS(200));
        }
        
        servo_generic_stop(continuous_servo);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

// Example of how to create and start servo tasks
void start_servo_examples(void)
{
    // Uncomment the example you want to run:
    
    // Position servo example
    // xTaskCreate(servo_position_example_task, "servo_pos", 4096, NULL, 5, NULL);
    
    // Continuous rotation servo example
    // xTaskCreate(servo_continuous_example_task, "servo_cont", 4096, NULL, 5, NULL);
    
    // Custom servo configuration example
    // xTaskCreate(servo_custom_example_task, "servo_custom", 4096, NULL, 5, NULL);
    
    // Multiple servos example
    // xTaskCreate(servo_multiple_example_task, "servo_multi", 4096, NULL, 5, NULL);
    
    ESP_LOGI(TAG, "Servo examples ready. Uncomment desired example in start_servo_examples()");
} 