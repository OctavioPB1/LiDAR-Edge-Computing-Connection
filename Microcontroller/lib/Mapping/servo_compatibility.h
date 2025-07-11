/**
 * @file servo_compatibility.h
 * @brief Simple compatibility layer for existing servo API
 * 
 * This file provides a simple compatibility layer that avoids conflicts
 * with the original servo library by using different names.
 * 
 * UPDATED FOR MS-R-1.3-9 SERVO:
 * - Centralized all servo specifications in this header
 * - Accurate speed calculations based on real servo specs
 * - Linear interpolation for precise angle tracking
 * - Calibration functions for fine-tuning
 * - All configuration in one place to avoid scattered variables
 * 
 * @version 2.0
 * @date 2025-01-27
 * @author Assistant
 */

#ifndef _SERVO_COMPATIBILITY_H_
#define _SERVO_COMPATIBILITY_H_

#include "esp_err.h"

// Need to include the servo_direction_t type
#include "Generic_servo/servo_generic.h"

// =============================================================================
// MS-R-1.3-9 SERVO SPECIFICATIONS (Centralized Configuration)
// =============================================================================
// Servo Model: MS-R-1.3-9
// Operating Voltage: 4.8V~6V (Using 5V)
// Operating Speed: 0.12sec/60degree (4.8V), 0.10sec/60degree (6V)
// At 5V: ~0.11sec/60degree = 545 degrees/second
// Required Pulse: 900us-2100us
// Operating Angle: 360degree continuous
// Direction: CCW (Counter-Clockwise)

#define SERVO_MS_R_1_3_9_MIN_PULSE_US       900     // Minimum pulse width
#define SERVO_MS_R_1_3_9_MAX_PULSE_US       2100    // Maximum pulse width
#define SERVO_MS_R_1_3_9_CENTER_PULSE_US    1440    // Center/stop pulse (calculated)
#define SERVO_MS_R_1_3_9_VOLTAGE            5.0f    // Operating voltage
#define SERVO_MS_R_1_3_9_MAX_SPEED_DPS      200.0f  // Reduced max speed for slower operation (was 545.0f)

// Speed calculation: Linear interpolation between center and extremes
// At center (1500us): 0 degrees/second
// At min (900us): -545 degrees/second (CW)
// At max (2100us): +545 degrees/second (CCW)

// Keep the existing API definitions for backward compatibility
#define SERVO_MAX_SPEED_CW      SERVO_MS_R_1_3_9_MIN_PULSE_US     // 900us
#define SERVO_MEDIUM_SPEED_CW   1200    // Medium speed for clockwise rotation
#define SERVO_LOW_SPEED_CW      1400    // Low speed for clockwise rotation (made slower)
#define SERVO_STOP              SERVO_MS_R_1_3_9_CENTER_PULSE_US  // 1440us
#define SERVO_LOW_SPEED_CCW     1480    // Low speed for counterclockwise rotation (made slower)
#define SERVO_MEDIUM_SPEED_CCW  1800    // Medium speed for counterclockwise rotation
#define SERVO_MAX_SPEED_CCW     SERVO_MS_R_1_3_9_MAX_PULSE_US     // 2100us

/**
 * @enum SERVO_DIRECTION_SIMPLE
 * @brief Defines the action of increase or decrease the speed of servo.
 */
typedef enum {
    SERVO_UP_SIMPLE,
    SERVO_DOWN_SIMPLE
} SERVO_DIRECTION_SIMPLE;


// Forward declarations of the compatibility API functions
esp_err_t servo_simple_initialize(void);
esp_err_t servo_simple_start(void);
esp_err_t servo_simple_stop(void);
esp_err_t servo_simple_pause(void);
esp_err_t servo_simple_restart(void);
int16_t readAngle_simple(void);
void servo_simple_reset_angle(void);
void servo_simple_set_speed(SERVO_DIRECTION_SIMPLE direction);
void servo_simple_invert(void);
esp_err_t delete_servo_simple_semaphores(void);

// New calibration and debugging functions
void servo_simple_calibrate_speed(uint32_t pulse_us, float measured_degrees_per_second);
void servo_simple_debug_angle_tracking(void);
int servo_simple_is_moving(void);
float servo_simple_get_angular_velocity(void);

// Compatibility macros for easy migration
#define servo_initialize() servo_simple_initialize()
#define servo_start() servo_simple_start()
#define servo_stop() servo_simple_stop()
#define servo_pause() servo_simple_pause()
#define servo_restart() servo_simple_restart()
#define readAngle() readAngle_simple()
#define servo_reset_angle() servo_simple_reset_angle()
#define servo_set_speed(dir) servo_simple_set_speed((dir == SERVO_UP_SIMPLE) ? SERVO_UP_SIMPLE : SERVO_DOWN_SIMPLE)
#define servo_invert() servo_simple_invert()
#define delete_servo_semaphores() delete_servo_simple_semaphores()

// Original SERVO_DIRECTION enum is already defined in servo.h

#endif /* _SERVO_COMPATIBILITY_SIMPLE_H_ */
