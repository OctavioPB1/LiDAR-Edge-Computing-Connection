/**
 * @file mapping.h
 * @brief Enhanced LiDAR mapping system for rotating servo applications
 * 
 * This module provides advanced mapping capabilities using VL53L0X sensor
 * with servo rotation, including data filtering, motion compensation,
 * and adaptive sampling for optimal mapping performance.
 * 
 * @version 2.0
 * @date 2025-02-09
 * @author Enhanced for servo-mounted LiDAR mapping
 */

#ifndef _MAPPING_H_
#define _MAPPING_H_

#include "esp_err.h"
#include <stdint.h>
#include <stdbool.h>

/**
 * @brief Mapping configuration structure
 * 
 * This structure contains all the parameters needed to configure the enhanced
 * mapping system. Each parameter affects the performance, accuracy, and behavior
 * of the LiDAR mapping system.
 */
typedef struct {
    uint32_t timing_budget_us;          /**< VL53L0X measurement timing budget in microseconds
                                         *   Controls how long the sensor takes to measure distance.
                                         *   Valid range: 15000-100000 us (15-100ms)
                                         *   - 15000 us (15ms): High speed, lower accuracy (~50Hz)
                                         *   - 20000 us (20ms): Balanced speed/accuracy (~40Hz)
                                         *   - 33000 us (33ms): Better accuracy (~25Hz)
                                         *   - 50000 us (50ms): High accuracy (~15Hz)
                                         *   - 100000 us (100ms): Maximum accuracy (~8Hz)
                                         *   Recommendation: Use 20000 for general mapping */
    
    uint32_t measurement_period_ms;     /**< Inter-measurement period in milliseconds
                                         *   Time between consecutive measurements in continuous mode.
                                         *   Valid range: 20-100 ms
                                         *   - Must be >= timing_budget_us/1000
                                         *   - Lower values = higher measurement rate
                                         *   - Higher values = lower power consumption
                                         *   Recommendation: Use 25ms for 40Hz operation */
    
    float servo_angular_velocity;       /**< Expected servo angular velocity in degrees per second
                                         *   CRITICAL PARAMETER: This is how fast your servo rotates.
                                         *   Valid range: 5.0-180.0 deg/s
                                         *   - 5-15 deg/s: Very slow, high precision mapping
                                         *   - 15-30 deg/s: Normal mapping speed
                                         *   - 30-60 deg/s: Fast mapping
                                         *   - 60-180 deg/s: Very fast mapping (may miss objects)
                                         *   MEASURE THIS: Use a stopwatch to time one full rotation
                                         *   Example: If servo takes 12 seconds for 180°, use 15.0 deg/s */
    
    uint16_t min_range_mm;             /**< Minimum valid range in millimeters
                                         *   Measurements below this distance are considered invalid.
                                         *   Valid range: 30-200 mm
                                         *   - 30-50 mm: Very close objects (may cause sensor saturation)
                                         *   - 50-100 mm: Normal close range
                                         *   - 100-200 mm: Conservative close range
                                         *   Recommendation: Use 50mm unless you need very close detection */
    
    uint16_t max_range_mm;             /**< Maximum valid range in millimeters
                                         *   Measurements above this distance are considered invalid.
                                         *   Valid range: 1000-4000 mm
                                         *   - 1000-1500 mm: Indoor mapping
                                         *   - 1500-2000 mm: General purpose
                                         *   - 2000-3000 mm: Large spaces
                                         *   - 3000-4000 mm: Maximum range (lower accuracy)
                                         *   Recommendation: Use 2000mm for most applications */
    
    uint8_t filter_window_size;        /**< Moving average filter window size
                                         *   Number of consecutive measurements to average for noise reduction.
                                         *   Valid range: 1-10
                                         *   - 1: No filtering (raw data)
                                         *   - 2-3: Light filtering, fast response
                                         *   - 4-6: Medium filtering, balanced
                                         *   - 7-10: Heavy filtering, slow response but stable
                                         *   Recommendation: Use 3 for most applications */
    
    bool motion_compensation_enabled;   /**< Enable robot motion compensation
                                         *   If true, corrects measurements based on robot movement.
                                         *   Requires calling mapping_set_robot_motion() with current velocities.
                                         *   - true: Use when robot is moving during mapping
                                         *   - false: Use when robot is stationary during mapping
                                         *   Recommendation: Enable if robot moves while mapping */
    
    bool adaptive_sampling_enabled;    /**< Enable adaptive sampling based on servo speed
                                         *   If true, adjusts measurement rate based on servo angular velocity.
                                         *   - true: Optimizes angular resolution automatically
                                         *   - false: Fixed measurement rate regardless of servo speed
                                         *   Recommendation: Enable for variable speed servos */
    
    float signal_rate_limit;           /**< Minimum signal rate threshold (0.0-1.0)
                                         *   Measurements with signal strength below this threshold are rejected.
                                         *   Valid range: 0.05-0.5
                                         *   - 0.05-0.1: Accept weak signals (more measurements, lower quality)
                                         *   - 0.1-0.2: Balanced signal quality
                                         *   - 0.2-0.5: High signal quality (fewer measurements, higher quality)
                                         *   Recommendation: Use 0.1 for most environments */
} mapping_config_t;

/**
 * @brief Enhanced measurement data structure
 */
typedef struct {
    uint16_t distance_mm;              /**< Filtered distance in millimeters */
    int16_t angle_deg;                 /**< Servo angle in degrees */
    uint16_t raw_distance_mm;          /**< Raw unfiltered distance */
    uint16_t signal_strength;          /**< Signal strength indicator */
    uint32_t timestamp_us;             /**< Measurement timestamp */
    float angular_velocity;            /**< Current servo angular velocity */
    bool valid;                        /**< True if measurement is valid */
    uint8_t quality_score;             /**< Data quality score (0-100) */
} mapping_measurement_t;

/**
 * @brief Mapping statistics structure
 */
typedef struct {
    uint32_t total_measurements;       /**< Total measurements taken */
    uint32_t valid_measurements;       /**< Valid measurements */
    uint32_t filtered_measurements;    /**< Measurements filtered out */
    uint32_t interpolated_points;      /**< Interpolated data points */
    float average_signal_strength;     /**< Average signal strength */
    float measurement_rate_hz;         /**< Current measurement rate */
    uint32_t error_count;              /**< Total error count */
} mapping_statistics_t;

/**
 * @brief Callback function type for new mapping data
 */
typedef void (*mapping_data_callback_t)(const mapping_measurement_t *measurement);

/**
 * @brief Initialize the enhanced mapping system
 * @param config Pointer to mapping configuration
 * @return ESP_OK on success, error code on failure
 */
esp_err_t mapping_init_enhanced(const mapping_config_t *config);

/**
 * @brief Configure mapping parameters
 * @param config Pointer to new configuration
 * @return ESP_OK on success, error code on failure
 */
esp_err_t mapping_configure(const mapping_config_t *config);

/**
 * @brief Start enhanced mapping with continuous mode
 * @param callback Optional callback function for new data
 * @return ESP_OK on success, error code on failure
 */
esp_err_t mapping_start_enhanced(mapping_data_callback_t callback);

/**
 * @brief Get enhanced mapping measurement with filtering and compensation
 * @param measurement Pointer to measurement structure
 * @return ESP_OK on success, error code on failure
 */
esp_err_t mapping_get_measurement_enhanced(mapping_measurement_t *measurement);

/**
 * @brief Get mapping statistics
 * @param stats Pointer to statistics structure
 * @return ESP_OK on success, error code on failure
 */
esp_err_t mapping_get_statistics(mapping_statistics_t *stats);

/**
 * @brief Enable/disable motion compensation
 * @param enable True to enable, false to disable
 * @return ESP_OK on success, error code on failure
 */
esp_err_t mapping_set_motion_compensation(bool enable);

/**
 * @brief Set robot motion vector for compensation
 * @param velocity_x Robot velocity in X direction (mm/s)
 * @param velocity_y Robot velocity in Y direction (mm/s)
 * @param angular_velocity Robot angular velocity (rad/s)
 * @return ESP_OK on success, error code on failure
 */
esp_err_t mapping_set_robot_motion(float velocity_x, float velocity_y, float angular_velocity);

/**
 * @brief Calibrate the mapping system
 * @param known_distance_mm Known distance for calibration
 * @return ESP_OK on success, error code on failure
 */
esp_err_t mapping_calibrate(uint16_t known_distance_mm);

/**
 * @brief Get interpolated measurement at specific angle
 * @param target_angle Target angle in degrees
 * @param measurement Pointer to interpolated measurement
 * @return ESP_OK on success, error code on failure
 */
esp_err_t mapping_get_interpolated_measurement(int16_t target_angle, mapping_measurement_t *measurement);

// Legacy compatibility functions
esp_err_t mapping_init(void);
esp_err_t getMappingValue(int16_t *angle, uint16_t *distance);
esp_err_t mapping_pause(void);
esp_err_t mapping_stop(void);
esp_err_t mapping_restart(void);

// Default configuration
extern const mapping_config_t MAPPING_CONFIG_DEFAULT;
extern const mapping_config_t MAPPING_CONFIG_HIGH_SPEED;
extern const mapping_config_t MAPPING_CONFIG_HIGH_ACCURACY;

// Example functions
void start_mapping_examples(void);

#endif /* _MAPPING_H_ */