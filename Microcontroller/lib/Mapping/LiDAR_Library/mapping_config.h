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
// PERFORMANCE TUNING - OPTIMIZED FOR YOUR PROJECT
// ============================================================================

/**
 * @brief Default timing budget for environment analysis
 * 
 * Time budget used during initial environment analysis.
 * Higher values = better accuracy but slower analysis.
 * 
 * Valid values: 20ms, 33ms, 50ms, 100ms, 200ms, 500ms
 * OPTIMIZED: 33ms for balance between speed and accuracy
 */
#define MAPPING_ANALYSIS_TIMING_BUDGET VL53L0X_TIMING_BUDGET_33MS

/**
 * @brief Number of measurements for environment analysis
 * 
 * More measurements = more accurate analysis but longer initialization time.
 * 
 * Range: 10-50
 * OPTIMIZED: 15 for faster initialization with good accuracy
 */
#define MAPPING_ANALYSIS_SAMPLES 15

/**
 * @brief Number of measurements for auto-calibration
 * 
 * More measurements = more accurate calibration but longer initialization time.
 * 
 * Range: 5-20
 * OPTIMIZED: 8 for faster startup
 */
#define MAPPING_CALIBRATION_SAMPLES 14

/**
 * @brief Minimum quality score for calibration measurements
 * 
 * Only measurements with quality score above this threshold will be used
 * for calibration calculations.
 * 
 * Range: 0-100
 * OPTIMIZED: 30 for more flexible calibration (was 60)
 */
#define MAPPING_CALIBRATION_MIN_QUALITY 30

// ============================================================================
// STATIC CALIBRATION WITH KNOWN DISTANCE (OPTIONAL)
// ============================================================================
/**
 * @brief Known distance for static calibration at startup (millimetres)
 *
 * Si colocas el robot a una distancia fija de una pared u objeto de referencia
 * antes de encenderlo, puedes indicar aquí esa distancia (en mm).   Si el valor
 * es > 0 la función auto_calibrate_system() llamará internamente a
 * mapping_calibrate() usando este patrón en lugar de la heurística.
 *
 * Ejemplo: para 500 mm fija =>
 *   #define MAPPING_KNOWN_CALIB_DISTANCE_MM 500
 *
 * Valor 0 desactiva esta función y mantiene el comportamiento heurístico.
 */
#define MAPPING_KNOWN_CALIB_DISTANCE_MM 500

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

// ===== CONFIGURACIONES PREDEFINIDAS PARA INTEGRACIÓN =====

/**
 * @brief Configuración optimizada para navegación de robot
 * Balance entre velocidad y precisión para navegación en tiempo real
 */
static const mapping_config_t MAPPING_CONFIG_NAVIGATION = {
    .timing_budget_us = 20000,          // 20ms - Balance velocidad/precisión  
    .measurement_period_ms = 25,        // 25ms - ~40Hz teórico
    .servo_angular_velocity = 30.0f,    // 30°/s - Velocidad moderada
    .min_range_mm = 50,                 // Evitar saturación cerca
    .max_range_mm = 2000,               // Rango útil navegación
    .filter_window_size = 3,            // Filtro ligero tiempo real
    .motion_compensation_enabled = true, // Compensar movimiento robot
    .adaptive_sampling_enabled = true,   // Sampling adaptativo
    .signal_rate_limit = 0.10f          // 10% threshold estándar
};

/**
 * @brief Configuración para detección de obstáculos
 * Optimizada para detección rápida de obstáculos cercanos
 */
static const mapping_config_t MAPPING_CONFIG_OBSTACLE_DETECTION = {
    .timing_budget_us = 15000,          // 15ms - Velocidad alta
    .measurement_period_ms = 20,        // 20ms - ~50Hz
    .servo_angular_velocity = 45.0f,    // 45°/s - Barrido rápido
    .min_range_mm = 30,                 // Detectar muy cerca
    .max_range_mm = 1000,               // Rango corto pero rápido
    .filter_window_size = 2,            // Filtro mínimo
    .motion_compensation_enabled = true, // Importante en movimiento
    .adaptive_sampling_enabled = true,   // Adaptativo
    .signal_rate_limit = 0.08f          // Threshold más bajo = más detecciones
};

/**
 * @brief Configuración para mapeo de precisión
 * Máxima precisión para mapeo detallado del entorno
 */
static const mapping_config_t MAPPING_CONFIG_PRECISION_MAPPING = {
    .timing_budget_us = 33000,          // 33ms - Máxima precisión
    .measurement_period_ms = 40,        // 40ms - Estable
    .servo_angular_velocity = 15.0f,    // 15°/s - Muy lento = más mediciones/grado
    .min_range_mm = 50,                 // Rango completo
    .max_range_mm = 4000,               // Máximo rango
    .filter_window_size = 5,            // Filtro pesado
    .motion_compensation_enabled = false, // Robot estático durante mapeo
    .adaptive_sampling_enabled = false,  // Sampling fijo
    .signal_rate_limit = 0.12f          // Threshold alto para calidad
};

/**
 * @brief Configuración para seguimiento de paredes
 * Optimizada para seguir paredes y contornos
 */
static const mapping_config_t MAPPING_CONFIG_WALL_FOLLOWING = {
    .timing_budget_us = 25000,          // 25ms - Precisión moderada
    .measurement_period_ms = 30,        // 30ms - ~33Hz
    .servo_angular_velocity = 20.0f,    // 20°/s - Velocidad controlada
    .min_range_mm = 40,                 // Muy cerca de paredes
    .max_range_mm = 1500,               // Rango medio
    .filter_window_size = 4,            // Filtro moderado
    .motion_compensation_enabled = true, // Robot en movimiento
    .adaptive_sampling_enabled = false,  // Sampling consistente
    .signal_rate_limit = 0.11f          // Buena calidad
};

/**
 * @brief Configuración para ahorro de energía
 * Consumo mínimo para operación con batería
 */
static const mapping_config_t MAPPING_CONFIG_POWER_SAVING = {
    .timing_budget_us = 10000,          // 10ms - Mínimo consumo
    .measurement_period_ms = 50,        // 50ms - ~20Hz
    .servo_angular_velocity = 20.0f,    // Velocidad moderada
    .min_range_mm = 100,                // Rango limitado
    .max_range_mm = 1500,               // Rango medio
    .filter_window_size = 2,            // Filtro mínimo
    .motion_compensation_enabled = false, // Menos procesamiento
    .adaptive_sampling_enabled = false,  // Fijo para predecibilidad
    .signal_rate_limit = 0.08f          // Threshold bajo
};

// ===== FIN CONFIGURACIONES PREDEFINIDAS =====

#ifdef __cplusplus
}
#endif

#endif // MAPPING_CONFIG_H 