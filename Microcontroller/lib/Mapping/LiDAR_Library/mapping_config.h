/**
 * @file mapping_config.h
 * @brief Configuration options for the enhanced mapping system
 * 
 * This file allows users to configure the automatic features of the mapping system
 * without modifying the main code. Simply uncomment or modify the desired options.
 */

#ifndef MAPPING_CONFIG_H
#define MAPPING_CONFIG_H

#include "mapping.h"

#ifdef __cplusplus
extern "C" {
#endif

// ============================================================================
// AUTOMATIC FEATURE CONFIGURATION
// ============================================================================

/**
 * @brief Enable automatic environment analysis
 * 
 * When enabled, the system will automatically analyze the environment quality
 * and adjust timing budget and filtering parameters accordingly.
 * 
 * Default: ENABLED
 */
#define MAPPING_AUTO_ENVIRONMENT_ANALYSIS 1

/**
 * @brief Enable automatic calibration
 * 
 * When enabled, the system will automatically calibrate itself using
 * environment analysis and median distance calculation.
 * 
 * Default: ENABLED
 */
#define MAPPING_AUTO_CALIBRATION 1

/**
 * @brief Enable automatic interpolation fallback
 * 
 * When enabled, if a measurement fails, the system will try to use
 * interpolated data from previous measurements.
 * 
 * Default: ENABLED
 */
#define MAPPING_AUTO_INTERPOLATION 1

/**
 * @brief Enable quality logging
 * 
 * When enabled, the system will log measurement quality information
 * every 5 seconds for debugging purposes.
 * 
 * Default: ENABLED
 */
#define MAPPING_QUALITY_LOGGING 1

/**
 * @brief Enable motion compensation by default
 * 
 * When enabled, motion compensation will be active from startup.
 * You can still disable it at runtime using mapping_set_motion_compensation(false).
 * 
 * Default: DISABLED (requires manual robot motion data)
 */
#define MAPPING_DEFAULT_MOTION_COMPENSATION 0

// ============================================================================
// PERFORMANCE TUNING
// ============================================================================

/**
 * @brief Default timing budget for environment analysis
 * 
 * Time budget used during initial environment analysis.
 * Higher values = better accuracy but slower analysis.
 * 
 * Valid values: 20ms, 33ms, 50ms, 100ms, 200ms, 500ms
 */
#define MAPPING_ANALYSIS_TIMING_BUDGET VL53L0X_TIMING_BUDGET_50MS

/**
 * @brief Number of measurements for environment analysis
 * 
 * More measurements = more accurate analysis but longer initialization time.
 * 
 * Range: 10-50
 */
#define MAPPING_ANALYSIS_SAMPLES 20

/**
 * @brief Number of measurements for auto-calibration
 * 
 * More measurements = more accurate calibration but longer initialization time.
 * 
 * Range: 5-20
 */
#define MAPPING_CALIBRATION_SAMPLES 10

/**
 * @brief Minimum quality score for calibration measurements
 * 
 * Only measurements with quality score above this threshold will be used
 * for calibration calculations.
 * 
 * Range: 0-100
 */
#define MAPPING_CALIBRATION_MIN_QUALITY 50

// ============================================================================
// LOGGING CONFIGURATION
// ============================================================================

/**
 * @brief Quality logging interval (microseconds)
 * 
 * How often to log quality information when MAPPING_QUALITY_LOGGING is enabled.
 * 
 * Default: 5 seconds (5,000,000 microseconds)
 */
#define MAPPING_QUALITY_LOG_INTERVAL_US 5000000

/**
 * @brief Enable detailed initialization logging
 * 
 * When enabled, detailed information about the initialization process
 * will be logged, including environment analysis results and calibration data.
 * 
 * Default: ENABLED
 */
#define MAPPING_DETAILED_INIT_LOGGING 1

// ============================================================================
// ADVANCED CONFIGURATION
// ============================================================================

/**
 * @brief Enable adaptive configuration
 * 
 * When enabled, the system will automatically adjust its configuration
 * based on runtime performance and measurement quality.
 * 
 * Default: DISABLED (experimental feature)
 */
#define MAPPING_ADAPTIVE_CONFIG 0

/**
 * @brief Enable performance monitoring
 * 
 * When enabled, the system will track and log performance metrics
 * such as measurement frequency, success rate, and processing time.
 * 
 * Default: DISABLED
 */
#define MAPPING_PERFORMANCE_MONITORING 0

// ============================================================================
// DEPRECATION WARNINGS
// ============================================================================

/**
 * @brief Show deprecation warnings for legacy functions
 * 
 * When enabled, warnings will be shown when legacy functions are used,
 * encouraging migration to enhanced functions.
 * 
 * Default: DISABLED
 */
#define MAPPING_SHOW_DEPRECATION_WARNINGS 0

#ifdef __cplusplus
}
#endif

#endif // MAPPING_CONFIG_H 