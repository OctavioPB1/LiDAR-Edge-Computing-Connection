/**
 * @file servo_angle_example.c
 * @brief Example demonstrating angle reading for continuous rotation servos
 * 
 * This example shows how to use the readAngle_simple() function to track
 * the angular position of a continuous rotation servo based on its movement.
 * 
 * @version 1.0
 * @date 2025-01-27
 * @author Assistant
 */

#include "servo_compatibility.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "SERVO_ANGLE_EXAMPLE";

void servo_angle_example_task(void *pvParameters)
{
    ESP_LOGI(TAG, "Starting servo angle reading example");
    
    // Initialize the servo
    esp_err_t ret = servo_simple_initialize();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize servo: %s", esp_err_to_name(ret));
        vTaskDelete(NULL);
        return;
    }
    
    // Start the servo
    ret = servo_simple_start();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start servo: %s", esp_err_to_name(ret));
        vTaskDelete(NULL);
        return;
    }
    
    // Reset angle to 0 degrees
    servo_simple_reset_angle();
    
    // Demonstrate angle reading during movement
    for (int i = 0; i < 10; i++) {
        // Read current angle
        int16_t current_angle = readAngle_simple();
        ESP_LOGI(TAG, "Current angle: %d degrees", current_angle);
        
        // Wait for 1 second
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    
    // Stop the servo
    servo_simple_stop();
    
    // Read final angle
    int16_t final_angle = readAngle_simple();
    ESP_LOGI(TAG, "Final angle: %d degrees", final_angle);
    
    // Demonstrate speed changes and angle tracking
    ESP_LOGI(TAG, "Testing speed changes...");
    
    // Start with medium speed
    servo_simple_start();
    vTaskDelay(pdMS_TO_TICKS(2000));
    
    // Increase speed
    servo_simple_set_speed(SERVO_UP_SIMPLE);
    vTaskDelay(pdMS_TO_TICKS(2000));
    
    // Decrease speed
    servo_simple_set_speed(SERVO_DOWN_SIMPLE);
    vTaskDelay(pdMS_TO_TICKS(2000));
    
    // Stop and read angle
    servo_simple_stop();
    int16_t angle_after_speed_changes = readAngle_simple();
    ESP_LOGI(TAG, "Angle after speed changes: %d degrees", angle_after_speed_changes);
    
    // Demonstrate angle reset
    ESP_LOGI(TAG, "Resetting angle to 0 degrees...");
    servo_simple_reset_angle();
    int16_t angle_after_reset = readAngle_simple();
    ESP_LOGI(TAG, "Angle after reset: %d degrees", angle_after_reset);
    
    // Clean up
    delete_servo_simple_semaphores();
    
    ESP_LOGI(TAG, "Servo angle reading example completed");
    vTaskDelete(NULL);
}

// Function to start the example (call this from main)
void start_servo_angle_example(void)
{
    xTaskCreate(servo_angle_example_task, "servo_angle_example", 4096, NULL, 5, NULL);
}