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
 */
typedef struct {
    uint32_t timing_budget_us;          /**< VL53L0X measurement timing budget */
    uint32_t measurement_period_ms;     /**< Inter-measurement period */
    float servo_angular_velocity;       /**< Expected servo angular velocity (deg/s) */
    uint16_t min_range_mm;             /**< Minimum valid range */
    uint16_t max_range_mm;             /**< Maximum valid range */
    uint8_t filter_window_size;        /**< Moving average filter window size */
    bool motion_compensation_enabled;   /**< Enable robot motion compensation */
    bool adaptive_sampling_enabled;    /**< Enable adaptive sampling based on servo speed */
    float signal_rate_limit;           /**< Minimum signal rate threshold */
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

#endif /* _MAPPING_H_ */