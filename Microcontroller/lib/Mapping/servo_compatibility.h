/**
 * @file servo_compatibility.h
 * @brief Compatibility layer for existing servo API using the new generic servo library
 * 
 * This file provides backward compatibility for the existing servo API while
 * using the new generic servo library internally. This allows for a smooth
 * migration without breaking existing code.
 * 
 * @version 1.0
 * @date 2025-01-27
 * @author Assistant
 */

#ifndef _SERVO_COMPATIBILITY_H_
#define _SERVO_COMPATIBILITY_H_

#include "esp_err.h"
#include "Generic_servo/servo_generic.h"

// Keep the existing API definitions for backward compatibility
#define SERVO_MAX_SPEED_CW      900     /**< Maximum speed for clockwise rotation. */
#define SERVO_MEDIUM_SPEED_CW   1300    /**< Medium speed for clockwise rotation. */
#define SERVO_LOW_SPEED_CW      1200    /**< Low speed for clockwise rotation. */
#define SERVO_STOP              1500    /**< PWM value to stop the servo. */
#define SERVO_LOW_SPEED_CCW     1800    /**< Low speed for counterclockwise rotation. */
#define SERVO_MEDIUM_SPEED_CCW  1650    /**< Medium speed for counterclockwise rotation. */
#define SERVO_MAX_SPEED_CCW     2100    /**< Maximum speed for counterclockwise rotation. */

/**
 * @enum SERVO_DIRECTION
 * @brief Defines the action of increase or decrease the speed of servo.
 */
typedef enum {
    UP,
    DOWN
} SERVO_DIRECTION;

// Forward declarations of the existing API functions
esp_err_t servo_initialize(void);
esp_err_t servo_start(void);
esp_err_t servo_stop(void);
esp_err_t servo_pause(void);
esp_err_t servo_restart(void);
int16_t readAngle(void);
void servo_set_speed(SERVO_DIRECTION direction);
void servo_invert(void);
esp_err_t delete_servo_semaphores(void);

// Additional helper functions for migration
esp_err_t servo_migrate_to_generic(void);
servo_handle_t* servo_get_generic_handle(void);

#endif /* _SERVO_COMPATIBILITY_H_ */ 