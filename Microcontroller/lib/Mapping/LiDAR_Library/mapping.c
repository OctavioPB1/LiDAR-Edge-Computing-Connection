/**
 * @file mapping.c
 * @brief Enhanced LiDAR mapping system implementation
 * 
 * This file implements advanced mapping capabilities using VL53L0X sensor
 * with servo rotation, including data filtering, motion compensation,
 * and adaptive sampling for optimal mapping performance.
 * 
 * @version 2.0
 * @date 2025-02-09
 * @author Enhanced for servo-mounted LiDAR mapping
 */

#include "mapping.h"
#include "mapping_config.h"
#include "servo_compatibility.h"
#include "vl53l0x.h"
#include "esp_log.h"
#include "debug_helper.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include <string.h>
#include <math.h>

#define MIN_DISTANCE 100
#define MAX_FILTER_WINDOW 10
#define INTERPOLATION_BUFFER_SIZE 360  // One measurement per degree
#define CALIBRATION_SAMPLES 20

static const char *TAG = "MAPPING_ENHANCED";

// Configuration presets
/**
 * @brief Default configuration preset - Balanced performance
 * 
 * This configuration provides a good balance between speed, accuracy, and power consumption.
 * Suitable for most general-purpose mapping applications.
 * 
 * Performance characteristics:
 * - Measurement rate: ~40Hz
 * - Servo speed: 30 deg/s (6 seconds for 180° rotation)
 * - Range: 50mm - 2000mm
 * - Light filtering for noise reduction
 * - Adaptive sampling enabled for optimal angular resolution
 * 
 * Use this preset for:
 * - General indoor mapping
 * - Robot navigation
 * - Object detection and avoidance
 * - When you're unsure which configuration to use
 */
const mapping_config_t MAPPING_CONFIG_DEFAULT = {
    .timing_budget_us = 20000,          // 20ms timing budget (~40Hz)
    .measurement_period_ms = 25,        // 25ms between measurements
    .servo_angular_velocity = 30.0f,    // 30 degrees per second
    .min_range_mm = 50,                 // 50mm minimum range
    .max_range_mm = 2000,               // 2000mm maximum range
    .filter_window_size = 3,            // 3-point moving average filter
    .motion_compensation_enabled = false, // Disabled by default
    .adaptive_sampling_enabled = true,  // Adaptive sampling enabled
    .signal_rate_limit = 0.1f           // 10% signal rate threshold
};

/**
 * @brief High-speed configuration preset - Maximum measurement rate
 * 
 * This configuration prioritizes speed over accuracy, achieving up to 50Hz measurement rate.
 * Best for applications requiring real-time response and fast mapping.
 * 
 * Performance characteristics:
 * - Measurement rate: ~50Hz
 * - Servo speed: 60 deg/s (3 seconds for 180° rotation)
 * - Range: 50mm - 1500mm (reduced for speed)
 * - Minimal filtering for fastest response
 * - Motion compensation enabled for moving robots
 * 
 * Use this preset for:
 * - Real-time obstacle avoidance
 * - Fast-moving robots
 * - Dynamic environments
 * - When response time is critical
 * - High-speed mapping applications
 */
const mapping_config_t MAPPING_CONFIG_HIGH_SPEED = {
    .timing_budget_us = 15000,          // 15ms timing budget (~50Hz)
    .measurement_period_ms = 20,        // 20ms between measurements
    .servo_angular_velocity = 60.0f,    // 60 degrees per second
    .min_range_mm = 50,                 // 50mm minimum range
    .max_range_mm = 1500,               // 1500mm maximum range (reduced for speed)
    .filter_window_size = 2,            // 2-point filter (minimal filtering)
    .motion_compensation_enabled = true, // Motion compensation enabled
    .adaptive_sampling_enabled = true,  // Adaptive sampling enabled
    .signal_rate_limit = 0.05f          // 5% signal rate threshold (more permissive)
};

/**
 * @brief High-accuracy configuration preset - Maximum precision
 * 
 * This configuration prioritizes accuracy and precision over speed.
 * Best for applications requiring detailed mapping and high measurement quality.
 * 
 * Performance characteristics:
 * - Measurement rate: ~15Hz
 * - Servo speed: 15 deg/s (12 seconds for 180° rotation)
 * - Range: 30mm - 2000mm (extended close range)
 * - Heavy filtering for maximum stability
 * - Motion compensation enabled
 * - Fixed sampling rate for consistent angular resolution
 * 
 * Use this preset for:
 * - Detailed environmental mapping
 * - Precision measurements
 * - Static mapping applications
 * - Quality-critical applications
 * - When accuracy is more important than speed
 * - Calibration and testing
 */
const mapping_config_t MAPPING_CONFIG_HIGH_ACCURACY = {
    .timing_budget_us = 50000,          // 50ms timing budget (~15Hz)
    .measurement_period_ms = 60,        // 60ms between measurements
    .servo_angular_velocity = 15.0f,    // 15 degrees per second (slow for precision)
    .min_range_mm = 30,                 // 30mm minimum range (very close detection)
    .max_range_mm = 2000,               // 2000mm maximum range
    .filter_window_size = 5,            // 5-point moving average filter (heavy filtering)
    .motion_compensation_enabled = true, // Motion compensation enabled
    .adaptive_sampling_enabled = false, // Fixed sampling for consistency
    .signal_rate_limit = 0.2f           // 20% signal rate threshold (high quality)
};

// Global state structure
static struct {
    mapping_config_t config;
    mapping_statistics_t stats;
    mapping_data_callback_t callback;
    
    // Filtering
    uint16_t filter_buffer[MAX_FILTER_WINDOW];
    uint8_t filter_index;
    bool filter_initialized;
    
    // Motion compensation
    struct {
        float velocity_x;
        float velocity_y;
        float angular_velocity;
        uint32_t last_update_time;
    } robot_motion;
    
    // Interpolation buffer
    mapping_measurement_t interpolation_buffer[INTERPOLATION_BUFFER_SIZE];
    bool interpolation_buffer_valid[INTERPOLATION_BUFFER_SIZE];
    
    // Calibration
    int16_t calibration_offset_mm;
    bool calibrated;
    
    // Synchronization
    SemaphoreHandle_t data_mutex;
    QueueHandle_t measurement_queue;
    TaskHandle_t mapping_task_handle;
    
    // State
    bool running;
    bool enhanced_mode;
    uint32_t last_measurement_time;
    
} mapping_state = {0};

// Internal function declarations
static esp_err_t getValue_enhanced(vl53l0x_measurement_t *measurement);
static uint16_t apply_moving_average_filter(uint16_t new_value);
static void apply_motion_compensation(mapping_measurement_t *measurement);
static uint8_t calculate_quality_score(const vl53l0x_measurement_t *raw_measurement);
static void update_statistics(const mapping_measurement_t *measurement);
static void store_interpolation_data(const mapping_measurement_t *measurement);
static void vl53l0x_data_ready_callback(vl53l0x_idx_t idx, vl53l0x_measurement_t *measurement);
static void enhanced_mapping_task(void *pvParameters);
static esp_err_t start_enhanced_measurement_task(void);
static esp_err_t stop_enhanced_measurement_task(void);
static esp_err_t auto_calibrate_system(void);
static esp_err_t analyze_environment_quality(void);

/**
 * @brief Initialize the enhanced mapping system
 */
esp_err_t mapping_init_enhanced(const mapping_config_t *config)
{
    esp_err_t err = ESP_OK;
    
    ESP_LOGI(TAG, "Initializing Enhanced Mapping System");
    
    // Initialize synchronization primitives
    mapping_state.data_mutex = xSemaphoreCreateMutex();
    if (!mapping_state.data_mutex) {
        ESP_LOGE(TAG, "Failed to create data mutex");
        return ESP_FAIL;
    }
    
    mapping_state.measurement_queue = xQueueCreate(10, sizeof(mapping_measurement_t));
    if (!mapping_state.measurement_queue) {
        ESP_LOGE(TAG, "Failed to create measurement queue");
        return ESP_FAIL;
    }
    
    // Initialize GPIO and I2C
    DEBUGING_ESP_LOG(ESP_LOGI(TAG, "Initializing GPIO..."));
    err = gpio_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error in gpio_init: %s", esp_err_to_name(err));
        LOG_MESSAGE_E(TAG, "Error in gpio_init");
        return err;
    }

    DEBUGING_ESP_LOG(ESP_LOGI(TAG, "Initializing I2C..."));
    err = i2c_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error initializing I2C");
        LOG_MESSAGE_E(TAG, "Error initializing I2C");
        return ESP_FAIL;
    }

    // Initialize VL53L0X
    DEBUGING_ESP_LOG(ESP_LOGI(TAG, "Initializing LiDAR..."));
    if (!vl53l0x_init()) {
        ESP_LOGE(TAG, "Error initializing LiDAR(VL53L0X)");
        LOG_MESSAGE_E(TAG, "Error initializing LiDAR(VL53L0X)");
        return ESP_FAIL;
    }
    
    // Configure VL53L0X timing budget
    if (config) {
        err = vl53l0x_set_timing_budget(VL53L0X_IDX_FIRST, config->timing_budget_us);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "Failed to set timing budget, using default");
        }
        
        // Set signal rate limit
        err = vl53l0x_set_signal_rate_limit(VL53L0X_IDX_FIRST, config->signal_rate_limit, 0.0f);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "Failed to set signal rate limit");
        }
    }

    // Initialize servo
    DEBUGING_ESP_LOG(ESP_LOGI(TAG, "Initializing ServoMotor..."));
    err = servo_initialize();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error initializing Servo");
        LOG_MESSAGE_E(TAG, "Error initializing Servo");
        return ESP_FAIL;
    }
    
    // Apply configuration
    if (config) {
        mapping_state.config = *config;
    } else {
        mapping_state.config = MAPPING_CONFIG_DEFAULT;
    }
    
    // Initialize state
    memset(&mapping_state.stats, 0, sizeof(mapping_statistics_t));
    memset(&mapping_state.robot_motion, 0, sizeof(mapping_state.robot_motion));
    memset(mapping_state.interpolation_buffer_valid, 0, sizeof(mapping_state.interpolation_buffer_valid));
    
    mapping_state.filter_index = 0;
    mapping_state.filter_initialized = false;
    mapping_state.calibrated = false;
    mapping_state.calibration_offset_mm = 0;
    mapping_state.enhanced_mode = true;
    
    ESP_LOGI(TAG, "Enhanced Mapping System initialized successfully");
    return ESP_OK;
}

/**
 * @brief Start enhanced mapping with continuous mode
 */
esp_err_t mapping_start_enhanced(mapping_data_callback_t callback)
{
    if (!mapping_state.enhanced_mode) {
        ESP_LOGE(TAG, "Enhanced mode not initialized");
        return ESP_FAIL;
    }
    
    esp_err_t err = ESP_OK;
    
    // Store callback
    mapping_state.callback = callback;
    
    // Start servo
    err = servo_start();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error starting servo");
        LOG_MESSAGE_E(TAG, "Error starting servo");
        return ESP_FAIL;
    }
    
    // Configure VL53L0X for continuous mode
    err = vl53l0x_start_continuous(VL53L0X_IDX_FIRST, mapping_state.config.measurement_period_ms);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start continuous mode");
        return ESP_FAIL;
    }
    
    // Install interrupt handler for data ready events
    err = vl53l0x_install_interrupt_handler(VL53L0X_IDX_FIRST, GPIO_NUM_2, vl53l0x_data_ready_callback);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Failed to install interrupt handler, using polling");
    }
    
    // Start measurement task
    err = start_enhanced_measurement_task();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start measurement task");
        return ESP_FAIL;
    }
    
    mapping_state.running = true;
    mapping_state.last_measurement_time = esp_timer_get_time();
    
    ESP_LOGI(TAG, "Enhanced mapping started");
    return ESP_OK;
}

/**
 * @brief Get enhanced mapping measurement with filtering and compensation
 */
esp_err_t mapping_get_measurement_enhanced(mapping_measurement_t *measurement)
{
    if (!measurement || !mapping_state.enhanced_mode) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Try to get measurement from queue (non-blocking)
    if (xQueueReceive(mapping_state.measurement_queue, measurement, 0) == pdTRUE) {
        return ESP_OK;
    }
    
    // If queue is empty, get direct measurement
    vl53l0x_measurement_t raw_measurement;
    esp_err_t err = getValue_enhanced(&raw_measurement);
    if (err != ESP_OK) {
        return err;
    }
    
    // Convert to enhanced measurement
    measurement->timestamp_us = raw_measurement.timestamp_us;
    measurement->angle_deg = readAngle();
    measurement->raw_distance_mm = raw_measurement.range_mm;
    measurement->signal_strength = raw_measurement.signal_rate;
    measurement->valid = raw_measurement.valid;
    
    // Apply calibration offset
    if (mapping_state.calibrated) {
        measurement->raw_distance_mm += mapping_state.calibration_offset_mm;
    }
    
    // Apply filtering
    if (measurement->valid) {
        measurement->distance_mm = apply_moving_average_filter(measurement->raw_distance_mm);
    } else {
        measurement->distance_mm = measurement->raw_distance_mm;
    }
    
    // Calculate angular velocity
    uint32_t current_time = esp_timer_get_time();
    if (mapping_state.last_measurement_time > 0) {
        // This is a simplified calculation - real implementation would track angle changes
        measurement->angular_velocity = mapping_state.config.servo_angular_velocity;
    } else {
        measurement->angular_velocity = 0.0f;
    }
    mapping_state.last_measurement_time = current_time;
    
    // Apply motion compensation
    if (mapping_state.config.motion_compensation_enabled) {
        apply_motion_compensation(measurement);
    }
    
    // Calculate quality score
    measurement->quality_score = calculate_quality_score(&raw_measurement);
    
    // Update statistics
    update_statistics(measurement);
    
    // Store for interpolation
    store_interpolation_data(measurement);
    
    return ESP_OK;
}

/**
 * @brief Enhanced measurement acquisition
 */
static esp_err_t getValue_enhanced(vl53l0x_measurement_t *measurement)
{
    if (!measurement) {
        return ESP_ERR_INVALID_ARG;
    }
    
    esp_err_t err = vl53l0x_read_range_continuous(VL53L0X_IDX_FIRST, measurement);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error reading continuous range: %s", esp_err_to_name(err));
        mapping_state.stats.error_count++;
        return err;
    }
    
    // Validate measurement
    if (measurement->range_mm < mapping_state.config.min_range_mm || 
        measurement->range_mm > mapping_state.config.max_range_mm) {
        measurement->valid = false;
        return ESP_ERR_INVALID_RESPONSE;
    }
    
    return ESP_OK;
}

/**
 * @brief Apply moving average filter
 */
static uint16_t apply_moving_average_filter(uint16_t new_value)
{
    if (mapping_state.config.filter_window_size == 0) {
        return new_value;
    }
    
    // Add new value to buffer
    mapping_state.filter_buffer[mapping_state.filter_index] = new_value;
    mapping_state.filter_index = (mapping_state.filter_index + 1) % mapping_state.config.filter_window_size;
    
    if (!mapping_state.filter_initialized && mapping_state.filter_index == 0) {
        mapping_state.filter_initialized = true;
    }
    
    // Calculate average
    uint32_t sum = 0;
    uint8_t count = mapping_state.filter_initialized ? mapping_state.config.filter_window_size : mapping_state.filter_index;
    
    for (uint8_t i = 0; i < count; i++) {
        sum += mapping_state.filter_buffer[i];
    }
    
    return (uint16_t)(sum / count);
}

/**
 * @brief Apply motion compensation
 */
static void apply_motion_compensation(mapping_measurement_t *measurement)
{
    if (!measurement->valid) {
        return;
    }
    
    uint32_t current_time = esp_timer_get_time();
    float time_diff_s = (current_time - mapping_state.robot_motion.last_update_time) / 1000000.0f;
    
    if (time_diff_s > 0.1f) { // Only compensate if motion data is recent
        return;
    }
    
    // Convert polar coordinates to cartesian for compensation
    float angle_rad = measurement->angle_deg * M_PI / 180.0f;
    float x = measurement->distance_mm * cos(angle_rad);
    float y = measurement->distance_mm * sin(angle_rad);
    
    // Apply robot motion compensation
    x += mapping_state.robot_motion.velocity_x * time_diff_s;
    y += mapping_state.robot_motion.velocity_y * time_diff_s;
    
    // Apply angular compensation
    float angle_compensation = mapping_state.robot_motion.angular_velocity * time_diff_s;
    float cos_comp = cos(angle_compensation);
    float sin_comp = sin(angle_compensation);
    
    float x_compensated = x * cos_comp - y * sin_comp;
    float y_compensated = x * sin_comp + y * cos_comp;
    
    // Convert back to polar coordinates
    measurement->distance_mm = (uint16_t)sqrtf(x_compensated * x_compensated + y_compensated * y_compensated);
    measurement->angle_deg = (int16_t)(atan2f(y_compensated, x_compensated) * 180.0f / M_PI);
}

/**
 * @brief Calculate measurement quality score
 */
static uint8_t calculate_quality_score(const vl53l0x_measurement_t *raw_measurement)
{
    if (!raw_measurement->valid) {
        return 0;
    }
    
    uint8_t score = 100;
    
    // Reduce score based on range status
    if (raw_measurement->range_status != 0) {
        score -= 30;
    }
    
    // Reduce score for low signal strength
    if (raw_measurement->signal_rate < (mapping_state.config.signal_rate_limit * 128)) {
        score -= 20;
    }
    
    // Reduce score for high ambient light
    if (raw_measurement->ambient_rate > 500) {
        score -= 15;
    }
    
    // Reduce score for out-of-range measurements
    if (raw_measurement->range_mm >= VL53L0X_OUT_OF_RANGE) {
        score = 0;
    }
    
    return score;
}

/**
 * @brief Update mapping statistics
 */
static void update_statistics(const mapping_measurement_t *measurement)
{
    if (xSemaphoreTake(mapping_state.data_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        mapping_state.stats.total_measurements++;
        
        if (measurement->valid) {
            mapping_state.stats.valid_measurements++;
            
            // Update running average of signal strength
            float alpha = 0.95f; // Exponential moving average factor
            mapping_state.stats.average_signal_strength = 
                alpha * mapping_state.stats.average_signal_strength + 
                (1.0f - alpha) * measurement->signal_strength;
        } else {
            mapping_state.stats.filtered_measurements++;
        }
        
        // Calculate measurement rate
        uint32_t current_time = esp_timer_get_time();
        static uint32_t last_rate_calculation = 0;
        static uint32_t measurements_since_last = 0;
        
        measurements_since_last++;
        
        if (current_time - last_rate_calculation >= 1000000) { // 1 second
            mapping_state.stats.measurement_rate_hz = measurements_since_last;
            measurements_since_last = 0;
            last_rate_calculation = current_time;
        }
        
        xSemaphoreGive(mapping_state.data_mutex);
    }
}

/**
 * @brief Store measurement data for interpolation
 */
static void store_interpolation_data(const mapping_measurement_t *measurement)
{
    if (!measurement->valid) {
        return;
    }
    
    // Normalize angle to 0-359 range
    int16_t normalized_angle = measurement->angle_deg;
    while (normalized_angle < 0) normalized_angle += 360;
    while (normalized_angle >= 360) normalized_angle -= 360;
    
    if (normalized_angle >= 0 && normalized_angle < INTERPOLATION_BUFFER_SIZE) {
        mapping_state.interpolation_buffer[normalized_angle] = *measurement;
        mapping_state.interpolation_buffer_valid[normalized_angle] = true;
    }
}

/**
 * @brief VL53L0X data ready callback
 */
static void vl53l0x_data_ready_callback(vl53l0x_idx_t idx, vl53l0x_measurement_t *measurement)
{
    // This callback is called from ISR context, so keep it minimal
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    
    if (mapping_state.mapping_task_handle) {
        vTaskNotifyGiveFromISR(mapping_state.mapping_task_handle, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

/**
 * @brief Enhanced mapping task
 */
static void enhanced_mapping_task(void *pvParameters)
{
    mapping_measurement_t measurement;
    TickType_t last_wake_time = xTaskGetTickCount();
    
    ESP_LOGI(TAG, "Enhanced mapping task started");
    
    while (mapping_state.running) {
        // Wait for notification or timeout
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(100));
        
        esp_err_t err = mapping_get_measurement_enhanced(&measurement);
        if (err == ESP_OK) {
            // Send to queue for other consumers
            xQueueSend(mapping_state.measurement_queue, &measurement, 0);
            
            // Call user callback if registered
            if (mapping_state.callback) {
                mapping_state.callback(&measurement);
            }
        }
        
        // Adaptive delay based on servo speed
        if (mapping_state.config.adaptive_sampling_enabled) {
            uint32_t delay_ms = (uint32_t)(1000.0f / (mapping_state.config.servo_angular_velocity / 2.0f));
            vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(delay_ms));
        } else {
            vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(mapping_state.config.measurement_period_ms));
        }
    }
    
    ESP_LOGI(TAG, "Enhanced mapping task stopped");
    vTaskDelete(NULL);
}

/**
 * @brief Start enhanced measurement task
 */
static esp_err_t start_enhanced_measurement_task(void)
{
    BaseType_t result = xTaskCreate(
        enhanced_mapping_task,
        "enhanced_mapping",
        4096,
        NULL,
        configMAX_PRIORITIES - 2,
        &mapping_state.mapping_task_handle
    );
    
    if (result != pdPASS) {
        ESP_LOGE(TAG, "Failed to create enhanced mapping task");
        return ESP_FAIL;
    }
    
    return ESP_OK;
}

// Additional enhanced functions implementation...

/**
 * @brief Get mapping statistics
 */
esp_err_t mapping_get_statistics(mapping_statistics_t *stats)
{
    if (!stats) {
        return ESP_ERR_INVALID_ARG;
    }
    
    if (xSemaphoreTake(mapping_state.data_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        *stats = mapping_state.stats;
        xSemaphoreGive(mapping_state.data_mutex);
        return ESP_OK;
    }
    
    return ESP_ERR_TIMEOUT;
}

/**
 * @brief Set robot motion for compensation
 */
esp_err_t mapping_set_robot_motion(float velocity_x, float velocity_y, float angular_velocity)
{
    mapping_state.robot_motion.velocity_x = velocity_x;
    mapping_state.robot_motion.velocity_y = velocity_y;
    mapping_state.robot_motion.angular_velocity = angular_velocity;
    mapping_state.robot_motion.last_update_time = esp_timer_get_time();
    
    return ESP_OK;
}

/**
 * @brief Enable/disable motion compensation
 */
esp_err_t mapping_set_motion_compensation(bool enable)
{
    mapping_state.config.motion_compensation_enabled = enable;
    ESP_LOGI(TAG, "Motion compensation %s", enable ? "enabled" : "disabled");
    return ESP_OK;
}

/**
 * @brief Calibrate the mapping system
 */
esp_err_t mapping_calibrate(uint16_t known_distance_mm)
{
    ESP_LOGI(TAG, "Starting calibration with known distance: %d mm", known_distance_mm);
    
    uint32_t sum = 0;
    uint16_t valid_samples = 0;
    
    for (int i = 0; i < CALIBRATION_SAMPLES; i++) {
        mapping_measurement_t measurement;
        esp_err_t err = mapping_get_measurement_enhanced(&measurement);
        
        if (err == ESP_OK && measurement.valid) {
            sum += measurement.raw_distance_mm;
            valid_samples++;
        }
        
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    
    if (valid_samples < CALIBRATION_SAMPLES / 2) {
        ESP_LOGE(TAG, "Calibration failed: insufficient valid samples");
        return ESP_FAIL;
    }
    
    uint16_t average_measured = sum / valid_samples;
    mapping_state.calibration_offset_mm = known_distance_mm - average_measured;
    mapping_state.calibrated = true;
    
    ESP_LOGI(TAG, "Calibration completed. Offset: %d mm", mapping_state.calibration_offset_mm);
    return ESP_OK;
}

/**
 * @brief Get interpolated measurement at specific angle
 */
esp_err_t mapping_get_interpolated_measurement(int16_t target_angle, mapping_measurement_t *measurement)
{
    if (!measurement) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Normalize angle
    while (target_angle < 0) target_angle += 360;
    while (target_angle >= 360) target_angle -= 360;
    
    // Check if we have data at this exact angle
    if (mapping_state.interpolation_buffer_valid[target_angle]) {
        *measurement = mapping_state.interpolation_buffer[target_angle];
        mapping_state.stats.interpolated_points++;
        return ESP_OK;
    }
    
    // Find nearest valid measurements for interpolation
    int16_t lower_angle = -1, upper_angle = -1;
    
    for (int16_t offset = 1; offset <= 10; offset++) {
        int16_t angle1 = (target_angle - offset + 360) % 360;
        int16_t angle2 = (target_angle + offset) % 360;
        
        if (lower_angle == -1 && mapping_state.interpolation_buffer_valid[angle1]) {
            lower_angle = angle1;
        }
        if (upper_angle == -1 && mapping_state.interpolation_buffer_valid[angle2]) {
            upper_angle = angle2;
        }
        
        if (lower_angle != -1 && upper_angle != -1) {
            break;
        }
    }
    
    if (lower_angle == -1 || upper_angle == -1) {
        return ESP_ERR_NOT_FOUND;
    }
    
    // Perform linear interpolation
    mapping_measurement_t *lower_meas = &mapping_state.interpolation_buffer[lower_angle];
    mapping_measurement_t *upper_meas = &mapping_state.interpolation_buffer[upper_angle];
    
    float weight = (float)(target_angle - lower_angle) / (upper_angle - lower_angle);
    
    measurement->angle_deg = target_angle;
    measurement->distance_mm = (uint16_t)(lower_meas->distance_mm * (1.0f - weight) + 
                                         upper_meas->distance_mm * weight);
    measurement->signal_strength = (uint16_t)(lower_meas->signal_strength * (1.0f - weight) + 
                                             upper_meas->signal_strength * weight);
    measurement->timestamp_us = esp_timer_get_time();
    measurement->valid = true;
    measurement->quality_score = (uint8_t)((lower_meas->quality_score + upper_meas->quality_score) / 2);
    
    mapping_state.stats.interpolated_points++;
    return ESP_OK;
}

// Legacy compatibility functions - Enhanced with automatic feature detection
esp_err_t mapping_init(void)
{
    ESP_LOGI(TAG, "Initializing mapping system with automatic enhancement detection");
    
    // Auto-detect optimal configuration based on environment
    mapping_config_t auto_config = MAPPING_CONFIG_DEFAULT;
    
    // Try to detect if we're in a high-speed environment
    // This could be based on servo speed detection or other heuristics
    bool high_speed_environment = false;
    
    // For now, we'll use the default configuration
    // In the future, this could be enhanced with:
    // - Servo speed detection
    // - Environment analysis
    // - Performance monitoring
    // - User preferences stored in NVS
    
    if (high_speed_environment) {
        auto_config = MAPPING_CONFIG_HIGH_SPEED;
        ESP_LOGI(TAG, "Auto-detected high-speed environment, using high-speed config");
    }
    
    // Initialize with auto-detected configuration
    esp_err_t err = mapping_init_enhanced(&auto_config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize enhanced mapping system");
        return err;
    }
    
    // Automatically start enhanced mode
    err = mapping_start_enhanced(NULL);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start enhanced mapping mode");
        return err;
    }
    
    // Perform automatic environment analysis and calibration (if enabled)
#if MAPPING_AUTO_ENVIRONMENT_ANALYSIS
    ESP_LOGI(TAG, "Performing automatic environment analysis...");
    analyze_environment_quality();
#endif

#if MAPPING_AUTO_CALIBRATION
    ESP_LOGI(TAG, "Performing automatic calibration...");
    auto_calibrate_system();
#endif
    
    // Configure motion compensation if enabled by default
#if MAPPING_DEFAULT_MOTION_COMPENSATION
    mapping_set_motion_compensation(true);
    ESP_LOGI(TAG, "Motion compensation enabled by default");
#endif

    ESP_LOGI(TAG, "Mapping system initialized with enhanced features enabled");
    ESP_LOGI(TAG, "Features: Continuous mode, filtering, quality assessment, interpolation, auto-calibration");
    
    return ESP_OK;
}

esp_err_t getMappingValue(int16_t *angle, uint16_t *distance)
{
    if (!angle || !distance) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Use enhanced measurement with all features enabled
    mapping_measurement_t measurement;
    esp_err_t err = mapping_get_measurement_enhanced(&measurement);
    
    if (err == ESP_OK && measurement.valid) {
        *angle = measurement.angle_deg;
        *distance = measurement.distance_mm;
        
#if MAPPING_QUALITY_LOGGING
        // Log quality information for debugging (optional)
        static uint32_t last_quality_log = 0;
        uint32_t current_time = esp_timer_get_time();
        if (current_time - last_quality_log > MAPPING_QUALITY_LOG_INTERVAL_US) {
            ESP_LOGD(TAG, "Measurement quality: %d%%, Signal: %d, Raw: %dmm, Filtered: %dmm", 
                    measurement.quality_score, 
                    measurement.signal_strength,
                    measurement.raw_distance_mm,
                    measurement.distance_mm);
            last_quality_log = current_time;
        }
#endif
    }
    
    return err;
}

esp_err_t mapping_pause(void)
{
    return servo_pause();
}

esp_err_t mapping_stop(void)
{
    mapping_state.running = false;
    stop_enhanced_measurement_task();
    vl53l0x_stop_continuous(VL53L0X_IDX_FIRST);
    return servo_stop();
}

esp_err_t mapping_restart(void)
{
    return servo_restart();
}

static esp_err_t stop_enhanced_measurement_task(void)
{
    if (mapping_state.mapping_task_handle) {
        mapping_state.running = false;
        // Task will delete itself
        mapping_state.mapping_task_handle = NULL;
    }
    return ESP_OK;
}

/**
 * @brief Auto-calibrate the system using environment analysis
 */
static esp_err_t auto_calibrate_system(void)
{
    ESP_LOGI(TAG, "Starting automatic calibration...");
    
    // Take multiple measurements to analyze environment
    uint32_t measurements[MAPPING_CALIBRATION_SAMPLES];
    uint16_t valid_count = 0;
    
    for (int i = 0; i < MAPPING_CALIBRATION_SAMPLES; i++) {
        mapping_measurement_t measurement;
        esp_err_t err = mapping_get_measurement_enhanced(&measurement);
        
        if (err == ESP_OK && measurement.valid && measurement.quality_score > MAPPING_CALIBRATION_MIN_QUALITY) {
            measurements[valid_count++] = measurement.raw_distance_mm;
        }
        
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    
    if (valid_count < MAPPING_CALIBRATION_SAMPLES / 2) {
        ESP_LOGW(TAG, "Auto-calibration failed: insufficient good measurements");
        return ESP_FAIL;
    }
    
    // Calculate median distance (more robust than average)
    for (int i = 0; i < valid_count - 1; i++) {
        for (int j = i + 1; j < valid_count; j++) {
            if (measurements[i] > measurements[j]) {
                uint32_t temp = measurements[i];
                measurements[i] = measurements[j];
                measurements[j] = temp;
            }
        }
    }
    
    uint32_t median_distance = measurements[valid_count / 2];
    
    // Estimate calibration offset based on typical indoor distances
    // This is a heuristic - in a real application, you might have known reference points
    int16_t estimated_offset = 0;
    
    if (median_distance < 500) {
        // Very close object, likely a wall or obstacle
        estimated_offset = 20; // Typical offset for close measurements
    } else if (median_distance < 2000) {
        // Medium distance, typical indoor environment
        estimated_offset = 10;
    } else {
        // Far distance, minimal offset
        estimated_offset = 5;
    }
    
    mapping_state.calibration_offset_mm = estimated_offset;
    mapping_state.calibrated = true;
    
    ESP_LOGI(TAG, "Auto-calibration completed. Median distance: %lu mm, Estimated offset: %d mm", 
             median_distance, estimated_offset);
    
    return ESP_OK;
}

/**
 * @brief Analyze environment quality and adjust configuration
 */
static esp_err_t analyze_environment_quality(void)
{
    ESP_LOGI(TAG, "Analyzing environment quality...");
    
    uint32_t total_quality = 0;
    uint16_t valid_measurements = 0;
    uint32_t avg_signal_strength = 0;
    
    // Take measurements to analyze environment
    for (int i = 0; i < MAPPING_ANALYSIS_SAMPLES; i++) {
        mapping_measurement_t measurement;
        esp_err_t err = mapping_get_measurement_enhanced(&measurement);
        
        if (err == ESP_OK && measurement.valid) {
            total_quality += measurement.quality_score;
            avg_signal_strength += measurement.signal_strength;
            valid_measurements++;
        }
        
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    
    if (valid_measurements < MAPPING_ANALYSIS_SAMPLES / 2) {
        ESP_LOGW(TAG, "Environment analysis failed: insufficient measurements");
        return ESP_FAIL;
    }
    
    uint32_t avg_quality = total_quality / valid_measurements;
    avg_signal_strength /= valid_measurements;
    
    ESP_LOGI(TAG, "Environment analysis: Avg quality: %lu%%, Avg signal: %lu", 
             avg_quality, avg_signal_strength);
    
    // Adjust configuration based on environment quality
    if (avg_quality < 30) {
        // Poor environment - increase timing budget for better accuracy
        ESP_LOGI(TAG, "Poor environment detected, increasing timing budget");
        mapping_state.config.timing_budget_us = VL53L0X_TIMING_BUDGET_100MS;
        mapping_state.config.filter_window_size = 8; // More aggressive filtering
    } else if (avg_quality > 80) {
        // Excellent environment - can use faster settings
        ESP_LOGI(TAG, "Excellent environment detected, optimizing for speed");
        mapping_state.config.timing_budget_us = VL53L0X_TIMING_BUDGET_20MS;
        mapping_state.config.filter_window_size = 3; // Less filtering needed
    }
    
    return ESP_OK;
}