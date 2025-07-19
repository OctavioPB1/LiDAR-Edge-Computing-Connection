/**
 * @file servo_generic.h
 * @brief Generic servo motor control library for ESP32
 * 
 * This library provides a unified interface for controlling different types of servo motors:
 * - Position servos (0-180 degrees)
 * - Continuous rotation servos
 * - Custom servo types with configurable parameters
 * 
 * Features:
 * - Support for multiple servo types
 * - Configurable pulse width ranges
 * - Speed control for continuous rotation servos
 * - Position control for standard servos
 * - Thread-safe operations
 * - Error handling and validation
 * 
 * @version 2.0
 * @date 2025-01-27
 * @author Assistant
 */

#ifndef _SERVO_GENERIC_H_
#define _SERVO_GENERIC_H_

#include "esp_err.h"
#include "driver/mcpwm_prelude.h"
#include <stdint.h>
#include <stdbool.h>
#include <soc/gpio_num.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Servo motor types
 */
typedef enum {
    SERVO_TYPE_POSITION,     /**< Standard position servo (0-180 degrees) */
    SERVO_TYPE_CONTINUOUS,   /**< Continuous rotation servo */
    SERVO_TYPE_CUSTOM        /**< Custom servo with user-defined parameters */
} servo_type_t;

/**
 * @brief Servo direction for continuous rotation
 */
typedef enum {
    SERVO_DIR_CW,    /**< Clockwise rotation */
    SERVO_DIR_CCW,   /**< Counter-clockwise rotation */
    SERVO_DIR_STOP   /**< Stop rotation */
} servo_direction_t;

/**
 * @brief Servo configuration structure
 */
typedef struct {
    gpio_num_t pin;                  /**< GPIO pin for PWM signal */
    servo_type_t type;               /**< Type of servo motor */
    uint32_t min_pulse_us;           /**< Minimum pulse width in microseconds */
    uint32_t max_pulse_us;           /**< Maximum pulse width in microseconds */
    uint32_t center_pulse_us;        /**< Center/stop pulse width in microseconds */
    uint32_t frequency_hz;           /**< PWM frequency in Hz (typically 50) */
    uint16_t min_angle;              /**< Minimum angle for position servos */
    uint16_t max_angle;              /**< Maximum angle for position servos */
    uint8_t speed;                   /**< Speed for continuous rotation (0-100) */
    bool invert_direction;           /**< Invert rotation direction */
} servo_config_t;

/**
 * @brief Servo handle structure
 */
typedef struct {
    servo_config_t config;           /**< Servo configuration */
    mcpwm_timer_handle_t timer;      /**< MCPWM timer handle */
    mcpwm_oper_handle_t operator;    /**< MCPWM operator handle */
    mcpwm_cmpr_handle_t comparator;  /**< MCPWM comparator handle */
    mcpwm_gen_handle_t generator;    /**< MCPWM generator handle */
    uint32_t current_pulse;          /**< Current pulse width */
    uint16_t current_angle;          /**< Current angle (for position servos) */
    uint8_t current_speed;           /**< Current speed (for continuous servos) */
    bool is_initialized;             /**< Initialization status */
    bool is_enabled;                 /**< Enable status */
} servo_handle_t;

/**
 * @brief Predefined servo configurations
 */
extern const servo_config_t SERVO_CONFIG_POSITION_STANDARD;    /**< Standard position servo (0-180°) */
extern const servo_config_t SERVO_CONFIG_POSITION_MG996R;      /**< MG996R position servo (0-180°) */
extern const servo_config_t SERVO_CONFIG_POSITION_SG90;        /**< SG90 position servo (0-180°) */
extern const servo_config_t SERVO_CONFIG_CONTINUOUS_STANDARD;  /**< Standard continuous rotation servo */
extern const servo_config_t SERVO_CONFIG_CONTINUOUS_FUTABA;    /**< Futaba continuous rotation servo */

/**
 * @brief Initialize a servo motor
 * 
 * @param config Servo configuration
 * @param handle Pointer to servo handle (will be allocated)
 * @return ESP_OK on success, error code on failure
 */
esp_err_t servo_generic_init(const servo_config_t *config, servo_handle_t **handle);

/**
 * @brief Deinitialize a servo motor
 * 
 * @param handle Servo handle
 * @return ESP_OK on success, error code on failure
 */
esp_err_t servo_generic_deinit(servo_handle_t *handle);

/**
 * @brief Enable servo motor
 * 
 * @param handle Servo handle
 * @return ESP_OK on success, error code on failure
 */
esp_err_t servo_generic_enable(servo_handle_t *handle);

/**
 * @brief Disable servo motor
 * 
 * @param handle Servo handle
 * @return ESP_OK on success, error code on failure
 */
esp_err_t servo_generic_disable(servo_handle_t *handle);

/**
 * @brief Set position for position servo (0-180 degrees)
 * 
 * @param handle Servo handle
 * @param angle Angle in degrees (0-180)
 * @return ESP_OK on success, error code on failure
 */
esp_err_t servo_generic_set_position(servo_handle_t *handle, uint16_t angle);

/**
 * @brief Set speed for continuous rotation servo
 * 
 * @param handle Servo handle
 * @param speed Speed (0-100, where 0=stop, 50=center, 100=full speed)
 * @param direction Rotation direction
 * @return ESP_OK on success, error code on failure
 */
esp_err_t servo_generic_set_speed(servo_handle_t *handle, uint8_t speed, servo_direction_t direction);

/**
 * @brief Stop servo motor
 * 
 * @param handle Servo handle
 * @return ESP_OK on success, error code on failure
 */
esp_err_t servo_generic_stop(servo_handle_t *handle);

/**
 * @brief Set custom pulse width
 * 
 * @param handle Servo handle
 * @param pulse_width Pulse width in microseconds
 * @return ESP_OK on success, error code on failure
 */
esp_err_t servo_generic_set_pulse(servo_handle_t *handle, uint32_t pulse_width);

/**
 * @brief Get current position/angle
 * 
 * @param handle Servo handle
 * @return Current angle in degrees (for position servos)
 */
uint16_t servo_generic_get_position(servo_handle_t *handle);

/**
 * @brief Get current speed
 * 
 * @param handle Servo handle
 * @return Current speed (0-100)
 */
uint8_t servo_generic_get_speed(servo_handle_t *handle);

/**
 * @brief Get current pulse width
 * 
 * @param handle Servo handle
 * @return Current pulse width in microseconds
 */
uint32_t servo_generic_get_pulse(servo_handle_t *handle);

/**
 * @brief Check if servo is enabled
 * 
 * @param handle Servo handle
 * @return true if enabled, false otherwise
 */
bool servo_generic_is_enabled(servo_handle_t *handle);

/**
 * @brief Invert servo direction
 * 
 * @param handle Servo handle
 * @return ESP_OK on success, error code on failure
 */
esp_err_t servo_generic_invert_direction(servo_handle_t *handle);

/**
 * @brief Set servo speed with direction for continuous rotation
 * 
 * @param handle Servo handle
 * @param speed Speed (0-100)
 * @param direction Direction (CW/CCW)
 * @return ESP_OK on success, error code on failure
 */
esp_err_t servo_generic_set_speed_direction(servo_handle_t *handle, uint8_t speed, servo_direction_t direction);

/**
 * @brief Smooth movement to target position
 * 
 * @param handle Servo handle
 * @param target_angle Target angle in degrees
 * @param duration_ms Movement duration in milliseconds
 * @return ESP_OK on success, error code on failure
 */
esp_err_t servo_generic_move_smooth(servo_handle_t *handle, uint16_t target_angle, uint32_t duration_ms);

/**
 * @brief Create custom servo configuration
 * 
 * @param pin GPIO pin
 * @param type Servo type
 * @param min_pulse Minimum pulse width in microseconds
 * @param max_pulse Maximum pulse width in microseconds
 * @param center_pulse Center/stop pulse width in microseconds
 * @param frequency PWM frequency in Hz
 * @param min_angle Minimum angle (for position servos)
 * @param max_angle Maximum angle (for position servos)
 * @return Servo configuration structure
 */
servo_config_t servo_generic_create_config(gpio_num_t pin, servo_type_t type, 
                                          uint32_t min_pulse, uint32_t max_pulse, uint32_t center_pulse,
                                          uint32_t frequency, uint16_t min_angle, uint16_t max_angle);

#ifdef __cplusplus
}
#endif

#endif /* _SERVO_GENERIC_H_ */ 