/**
 * @file servo_compatibility.h
 * @brief Simple compatibility layer for existing servo API
 * 
 * This file provides a simple compatibility layer that avoids conflicts
 * with the original servo library by using different names.
 * 
 * UPDATED FOR SG90 SERVO:
 * - Uses SG90_CONFIG from main.c
 * - Position-based control (0-180 degrees)
 * - Continuous sweep simulation using servo_generic_move_smooth()
 * - Only includes functions used by other libraries
 * 
 * @version 4.0
 * @date 2025-01-27
 * @author Assistant
 */

#ifndef _SERVO_COMPATIBILITY_H_
#define _SERVO_COMPATIBILITY_H_

#include "esp_err.h"

// Need to include the servo_direction_t type
#include "Generic_Library/servo_generic.h"

// =============================================================================
// SG90 SERVO SPECIFICATIONS (From main.c SG90_CONFIG)
// =============================================================================
// Servo Model: SG90
// Operating Voltage: 4.8V~6V (Using 5V)
// Operating Speed: 0.1sec/60degree (4.8V), 0.08sec/60degree (6V)
// Stall Torque: 1.8kg.cm (4.8V), 2.5kg.cm (6V)
// Operating Angle: 0-180 degrees (Position servo)
// Required Pulse: 350us-2000us (from main.c)
// Frequency: 50Hz
// Type: Position servo (not continuous rotation)

#define SERVO_SG90_MIN_PULSE_US       450     // SG90 minimum pulse width (from main.c)
#define SERVO_SG90_MAX_PULSE_US       2150    // SG90 maximum pulse width (from main.c)
#define SERVO_SG90_CENTER_PULSE_US    1000    // SG90 center pulse (90 degrees, from main.c)
#define SERVO_SG90_VOLTAGE            5.0f    // Operating voltage
#define SERVO_SG90_MIN_ANGLE          0       // Minimum angle
#define SERVO_SG90_MAX_ANGLE          180     // Maximum angle
#define SERVO_SG90_FREQUENCY_HZ       50      // Standard servo frequency

// Position-based API definitions for SG90
#define SERVO_POSITION_MIN            0       // 0 degrees
#define SERVO_POSITION_CENTER         90      // 90 degrees (center)
#define SERVO_POSITION_MAX            180     // 180 degrees

static const servo_config_t SG90_CONFIG = {
    .pin = GPIO_NUM_14,
    .type = SERVO_TYPE_POSITION,
    .min_pulse_us = SERVO_SG90_MIN_PULSE_US,      // SG90 minimum pulse (from main.c)
    .max_pulse_us = SERVO_SG90_MAX_PULSE_US,     // SG90 maximum pulse (from main.c)
    .center_pulse_us = SERVO_SG90_CENTER_PULSE_US,  // SG90 center pulse (90 degrees, from main.c)
    .frequency_hz = SERVO_SG90_FREQUENCY_HZ,       // Standard servo frequency
    .min_angle = SERVO_SG90_MIN_ANGLE,           // Minimum angle
    .max_angle = SERVO_SG90_MAX_ANGLE,         // Maximum angle
    .speed = 50,              // Default speed (not used for position servos)
    .invert_direction = false // Normal direction
};

/**
 * @enum SERVO_DIRECTION_SIMPLE
 * @brief Defines the action of increase or decrease the position of servo.
 */
typedef enum {
    SERVO_UP_SIMPLE,      // Move towards higher angle (0->180)
    SERVO_DOWN_SIMPLE     // Move towards lower angle (180->0)
} SERVO_DIRECTION_SIMPLE;

// Forward declarations of the compatibility API functions (only those used by other libraries)
esp_err_t servo_simple_initialize(void);
esp_err_t servo_simple_start(void);
esp_err_t servo_simple_stop(void);
esp_err_t servo_simple_pause(void);
esp_err_t servo_simple_restart(void);
int16_t readAngle_simple(void);
void servo_simple_set_speed(SERVO_DIRECTION_SIMPLE direction);
void servo_simple_invert(void);
esp_err_t delete_servo_simple_semaphores(void);

// Additional functions used by main.c
esp_err_t servo_simple_set_position(uint16_t angle);
bool servo_simple_is_enabled(void);

// New function for continuous sweep simulation
esp_err_t servo_simple_continuous_sweep(void);

// Compatibility macros for easy migration
#define servo_initialize() servo_simple_initialize()
#define servo_start() servo_simple_start()
#define servo_stop() servo_simple_stop()
#define servo_pause() servo_simple_pause()
#define servo_restart() servo_simple_restart()
#define readAngle() readAngle_simple()
#define servo_set_speed(dir) servo_simple_set_speed((dir == SERVO_UP_SIMPLE) ? SERVO_UP_SIMPLE : SERVO_DOWN_SIMPLE)
#define servo_invert() servo_simple_invert()
#define delete_servo_semaphores() delete_servo_simple_semaphores()
#define servo_continuous_sweep() servo_simple_continuous_sweep()

// Original SERVO_DIRECTION enum is already defined in servo.h

#endif /* _SERVO_COMPATIBILITY_H_ */
