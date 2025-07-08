/**
 * @file mapping_example.c
 * @brief Example usage of the enhanced VL53L0X mapping system
 * 
 * This file demonstrates how to use the enhanced mapping capabilities
 * including continuous mode, data filtering, motion compensation,
 * and advanced features for servo-mounted LiDAR applications.
 * 
 * @version 1.0
 * @date 2025-02-09
 * @author Enhanced mapping example
 */

#include "mapping.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include <math.h>

static const char *TAG = "MAPPING_EXAMPLE";

// Example callback function for processing new mapping data
static void mapping_data_received(const mapping_measurement_t *measurement)
{
    if (measurement->valid) {
        ESP_LOGI(TAG, "New measurement: Angle=%d°, Distance=%dmm, Quality=%d%%, Signal=%d",
                measurement->angle_deg,
                measurement->distance_mm,
                measurement->quality_score,
                measurement->signal_strength);
        
        // Here you can process the measurement data
        // For example: send via WiFi, store in buffer, etc.
    } else {
        ESP_LOGW(TAG, "Invalid measurement received");
    }
}

/**
 * @brief Example 1: Basic enhanced mapping usage
 */
void mapping_example_basic(void)
{
    ESP_LOGI(TAG, "=== Basic Enhanced Mapping Example ===");
    
    // Initialize with default configuration
    esp_err_t err = mapping_init_enhanced(&MAPPING_CONFIG_DEFAULT);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize mapping system");
        return;
    }
    
    // Start enhanced mapping with callback
    err = mapping_start_enhanced(mapping_data_received);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start enhanced mapping");
        return;
    }
    
    ESP_LOGI(TAG, "Enhanced mapping started. Collecting data for 30 seconds...");
    
    // Let it run for 30 seconds
    vTaskDelay(pdMS_TO_TICKS(30000));
    
    // Get statistics
    mapping_statistics_t stats;
    err = mapping_get_statistics(&stats);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "Mapping Statistics:");
        ESP_LOGI(TAG, "  Total measurements: %lu", stats.total_measurements);
        ESP_LOGI(TAG, "  Valid measurements: %lu", stats.valid_measurements);
        ESP_LOGI(TAG, "  Filtered measurements: %lu", stats.filtered_measurements);
        ESP_LOGI(TAG, "  Measurement rate: %.1f Hz", stats.measurement_rate_hz);
        ESP_LOGI(TAG, "  Average signal strength: %.1f", stats.average_signal_strength);
        ESP_LOGI(TAG, "  Error count: %lu", stats.error_count);
    }
    
    // Stop mapping
    mapping_stop();
    ESP_LOGI(TAG, "Basic example completed");
}

/**
 * @brief Example 2: High-speed mapping configuration
 */
void mapping_example_high_speed(void)
{
    ESP_LOGI(TAG, "=== High-Speed Mapping Example ===");
    
    // Initialize with high-speed configuration
    esp_err_t err = mapping_init_enhanced(&MAPPING_CONFIG_HIGH_SPEED);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize high-speed mapping");
        return;
    }
    
    // Enable motion compensation for moving robot
    mapping_set_motion_compensation(true);
    
    // Simulate robot motion (moving forward at 100mm/s)
    mapping_set_robot_motion(100.0f, 0.0f, 0.0f);
    
    // Start mapping
    err = mapping_start_enhanced(mapping_data_received);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start high-speed mapping");
        return;
    }
    
    ESP_LOGI(TAG, "High-speed mapping with motion compensation started");
    
    // Update robot motion periodically
    for (int i = 0; i < 20; i++) {
        // Simulate changing robot velocity
        float velocity_x = 100.0f * cos(i * 0.1f);
        float velocity_y = 50.0f * sin(i * 0.1f);
        float angular_vel = 0.1f * sin(i * 0.2f);
        
        mapping_set_robot_motion(velocity_x, velocity_y, angular_vel);
        
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    
    mapping_stop();
    ESP_LOGI(TAG, "High-speed example completed");
}

/**
 * @brief Example 3: Manual measurement reading with interpolation
 */
void mapping_example_manual_reading(void)
{
    ESP_LOGI(TAG, "=== Manual Reading with Interpolation Example ===");
    
    // Initialize with high accuracy configuration
    esp_err_t err = mapping_init_enhanced(&MAPPING_CONFIG_HIGH_ACCURACY);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize mapping system");
        return;
    }
    
    // Start mapping without callback
    err = mapping_start_enhanced(NULL);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start mapping");
        return;
    }
    
    // Collect some data first
    ESP_LOGI(TAG, "Collecting initial data...");
    vTaskDelay(pdMS_TO_TICKS(10000));
    
    // Manually read measurements at specific intervals
    ESP_LOGI(TAG, "Reading measurements manually:");
    
    for (int angle = 0; angle < 360; angle += 10) {
        mapping_measurement_t measurement;
        
        // Try to get interpolated measurement at specific angle
        err = mapping_get_interpolated_measurement(angle, &measurement);
        
        if (err == ESP_OK) {
            ESP_LOGI(TAG, "Angle %d°: Distance=%dmm (interpolated)", 
                    angle, measurement.distance_mm);
        } else {
            // If interpolation fails, try direct reading
            err = mapping_get_measurement_enhanced(&measurement);
            if (err == ESP_OK && measurement.valid) {
                ESP_LOGI(TAG, "Angle %d°: Distance=%dmm (direct, actual angle=%d°)", 
                        angle, measurement.distance_mm, measurement.angle_deg);
            } else {
                ESP_LOGW(TAG, "No data available for angle %d°", angle);
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    
    mapping_stop();
    ESP_LOGI(TAG, "Manual reading example completed");
}

/**
 * @brief Example 4: Calibration and custom configuration
 */
void mapping_example_calibration(void)
{
    ESP_LOGI(TAG, "=== Calibration Example ===");
    
    // Create custom configuration optimized for calibration
    mapping_config_t custom_config = {
        .timing_budget_us = 30000,          // 30ms timing budget for high accuracy
        //   - Longer timing = better accuracy for calibration
        //   - 30ms provides good balance between speed and precision
        //   - Use 50ms or 100ms for maximum calibration accuracy
        
        .measurement_period_ms = 40,        // 40ms between measurements
        //   - Must be >= timing_budget_us/1000 (30ms)
        //   - 40ms gives ~25Hz measurement rate
        //   - Slower rate allows for more stable measurements during calibration
        
        .servo_angular_velocity = 20.0f,    // Slow servo speed for precision
        //   - 20 deg/s = 9 seconds for 180° rotation
        //   - Slower speed = more measurements per degree = better angular resolution
        //   - Critical for accurate calibration measurements
        
        .min_range_mm = 30,                 // Very close range detection
        //   - 30mm minimum allows calibration at close distances
        //   - Useful for calibrating with nearby objects
        //   - May cause sensor saturation if too close
        
        .max_range_mm = 3000,               // Extended maximum range
        //   - 3000mm allows calibration at various distances
        //   - Useful for testing calibration across different ranges
        //   - Higher range may reduce accuracy at close distances
        
        .filter_window_size = 5,            // Heavy filtering for stability
        //   - 5-point moving average filter
        //   - Reduces noise for more consistent calibration
        //   - Slower response but more stable measurements
        
        .motion_compensation_enabled = true, // Enable motion compensation
        //   - Corrects for any robot movement during calibration
        //   - Important if robot might move during calibration process
        //   - Set to false if robot is completely stationary
        
        .adaptive_sampling_enabled = false, // Fixed sampling rate
        //   - Disabled for consistent measurement intervals during calibration
        //   - Ensures predictable timing for calibration calculations
        //   - Use true for variable speed servos in normal operation
        
        .signal_rate_limit = 0.15f          // Higher signal quality threshold
        //   - 15% signal rate threshold (higher than default 10%)
        //   - Rejects weak signals that could affect calibration accuracy
        //   - May result in fewer measurements but higher quality
    };
    
    // Initialize with custom configuration
    esp_err_t err = mapping_init_enhanced(&custom_config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize mapping system");
        return;
    }
    
    // Start mapping
    err = mapping_start_enhanced(NULL);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start mapping");
        return;
    }
    
    ESP_LOGI(TAG, "Place a known object at exactly 500mm from the sensor");
    ESP_LOGI(TAG, "Starting calibration in 5 seconds...");
    vTaskDelay(pdMS_TO_TICKS(5000));
    
    // Perform calibration with known distance
    err = mapping_calibrate(500);  // Known distance of 500mm
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "Calibration successful!");
    } else {
        ESP_LOGE(TAG, "Calibration failed");
    }
    
    // Test calibrated measurements
    ESP_LOGI(TAG, "Testing calibrated measurements:");
    
    for (int i = 0; i < 10; i++) {
        mapping_measurement_t measurement;
        err = mapping_get_measurement_enhanced(&measurement);
        
        if (err == ESP_OK && measurement.valid) {
            ESP_LOGI(TAG, "Measurement %d: Raw=%dmm, Calibrated=%dmm, Quality=%d%%",
                    i + 1,
                    measurement.raw_distance_mm,
                    measurement.distance_mm,
                    measurement.quality_score);
        }
        
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    
    mapping_stop();
    ESP_LOGI(TAG, "Calibration example completed");
}

/**
 * @brief Example 5: Performance comparison
 */
void mapping_example_performance_comparison(void)
{
    ESP_LOGI(TAG, "=== Performance Comparison Example ===");
    
    mapping_statistics_t stats_enhanced;
    uint32_t start_time, end_time;
    
    // Test legacy mode
    ESP_LOGI(TAG, "Testing legacy mapping mode...");
    start_time = esp_timer_get_time();
    
    esp_err_t err = mapping_init();  // Legacy initialization
    if (err == ESP_OK) {
        // Simulate legacy usage
        for (int i = 0; i < 100; i++) {
            int16_t angle;
            uint16_t distance;
            getMappingValue(&angle, &distance);
            vTaskDelay(pdMS_TO_TICKS(50));  // 20Hz sampling
        }
        mapping_stop();
    }
    
    end_time = esp_timer_get_time();
    uint32_t legacy_time = (end_time - start_time) / 1000;  // Convert to ms
    
    // Test enhanced mode
    ESP_LOGI(TAG, "Testing enhanced mapping mode...");
    start_time = esp_timer_get_time();
    
    err = mapping_init_enhanced(&MAPPING_CONFIG_HIGH_SPEED);
    if (err == ESP_OK) {
        mapping_start_enhanced(NULL);
        
        // Simulate enhanced usage
        for (int i = 0; i < 100; i++) {
            mapping_measurement_t measurement;
            mapping_get_measurement_enhanced(&measurement);
            vTaskDelay(pdMS_TO_TICKS(20));  // 50Hz sampling
        }
        
        mapping_get_statistics(&stats_enhanced);
        mapping_stop();
    }
    
    end_time = esp_timer_get_time();
    uint32_t enhanced_time = (end_time - start_time) / 1000;  // Convert to ms
    
    // Compare results
    ESP_LOGI(TAG, "Performance Comparison Results:");
    ESP_LOGI(TAG, "  Legacy mode time: %lu ms", legacy_time);
    ESP_LOGI(TAG, "  Enhanced mode time: %lu ms", enhanced_time);
    ESP_LOGI(TAG, "  Enhanced mode measurements: %lu", stats_enhanced.total_measurements);
    ESP_LOGI(TAG, "  Enhanced mode valid rate: %.1f%%", 
            (float)stats_enhanced.valid_measurements / stats_enhanced.total_measurements * 100.0f);
    ESP_LOGI(TAG, "  Enhanced mode measurement rate: %.1f Hz", stats_enhanced.measurement_rate_hz);
    
    ESP_LOGI(TAG, "Performance comparison completed");
}

/**
 * @brief Main example task that runs all examples
 */
void mapping_examples_task(void *pvParameters)
{
    ESP_LOGI(TAG, "Starting VL53L0X Enhanced Mapping Examples");
    
    // Run all examples in sequence
    mapping_example_basic();
    vTaskDelay(pdMS_TO_TICKS(2000));
    
    mapping_example_high_speed();
    vTaskDelay(pdMS_TO_TICKS(2000));
    
    mapping_example_manual_reading();
    vTaskDelay(pdMS_TO_TICKS(2000));
    
    mapping_example_calibration();
    vTaskDelay(pdMS_TO_TICKS(2000));
    
    mapping_example_performance_comparison();
    
    ESP_LOGI(TAG, "All examples completed!");
    
    // Delete this task
    vTaskDelete(NULL);
}

/**
 * @brief Start the mapping examples
 */
void start_mapping_examples(void)
{
    xTaskCreate(
        mapping_examples_task,
        "mapping_examples",
        8192,  // Large stack for examples
        NULL,
        tskIDLE_PRIORITY + 1,
        NULL
    );
} 