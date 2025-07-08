/**
 * @file servo_compatibility.h
 * @brief Simple compatibility layer for existing servo API
 * 
 * This file provides a simple compatibility layer that avoids conflicts
 * with the original servo library by using different names.
 * 
 * @version 1.0
 * @date 2025-01-27
 * @author Assistant
 */

#ifndef _SERVO_COMPATIBILITY_H_
#define _SERVO_COMPATIBILITY_H_

#include "esp_err.h"

// Keep the existing API definitions for backward compatibility
#define SERVO_MAX_SPEED_CW      900     /**< Maximum speed for clockwise rotation. */
#define SERVO_MEDIUM_SPEED_CW   1300    /**< Medium speed for clockwise rotation. */
#define SERVO_LOW_SPEED_CW      1200    /**< Low speed for clockwise rotation. */
#define SERVO_STOP              1500    /**< PWM value to stop the servo. */
#define SERVO_LOW_SPEED_CCW     1800    /**< Low speed for counterclockwise rotation. */
#define SERVO_MEDIUM_SPEED_CCW  1650    /**< Medium speed for counterclockwise rotation. */
#define SERVO_MAX_SPEED_CCW     2100    /**< Maximum speed for counterclockwise rotation. */

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
