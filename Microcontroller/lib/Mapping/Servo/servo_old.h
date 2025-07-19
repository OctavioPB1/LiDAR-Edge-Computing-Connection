// /**
//  * @file servo.h
//  * @author Guerrico Leonel (lguerrico@outlook.com)
//  * @brief Servo motor control module.
//  *
//  * This file provides function prototypes and definitions for controlling a servo motor, 
//  * including speed settings, direction control, and operational states.
//  *
//  * The servo is controlled via PWM signals with predefined speed values for clockwise (CW) 
//  * and counterclockwise (CCW) rotations.
//  *
//  * @date 2025-02-09
//  */

// #ifndef _SERVO_H_
// #define _SERVO_H_

// #include "esp_err.h"

// /** @defgroup Servo Speed Definitions
//  *  Predefined PWM values for different servo speeds.
//  */
// #define SERVO_MAX_SPEED_CW      900     /**< Maximum speed for clockwise rotation. */
// #define SERVO_MEDIUM_SPEED_CW   1300    /**< Medium speed for clockwise rotation (previously 1050). */
// #define SERVO_LOW_SPEED_CW      1200    /**< Low speed for clockwise rotation (tested). */
// #define SERVO_STOP              1500    /**< PWM value to stop the servo. */
// #define SERVO_LOW_SPEED_CCW     1800    /**< Low speed for counterclockwise rotation (tested). */
// #define SERVO_MEDIUM_SPEED_CCW  1650    /**< Medium speed for counterclockwise rotation (previously 1950). */
// #define SERVO_MAX_SPEED_CCW     2100    /**< Maximum speed for counterclockwise rotation. */


// /**
//  * @enum SERVO_DIRECTION
//  * @brief Defines the action of increase or decrease the speed of servo.
//  */
// typedef enum {
//     UP,
//     DOWN
// } SERVO_DIRECTION;



// /**
//  * @brief Initializes the servo motor.
//  *
//  * This function sets up the necessary configurations for the servo to operate.
//  *
//  * @return ESP_OK on successful initialization, ESP_FAIL on failure.
//  */
// esp_err_t servo_initialize(void);

// /**
//  * @brief Starts the servo motor.
//  *
//  * This function enables the servo operation at its current speed.
//  *
//  * @return ESP_OK on success, ESP_FAIL on failure.
//  */
// esp_err_t servo_start(void);

// /**
//  * @brief Stops the servo motor.
//  *
//  * This function halts the servo by setting the PWM signal to the stop value.
//  *
//  * @return ESP_OK on success, ESP_FAIL on failure.
//  */
// esp_err_t servo_stop(void);

// /**
//  * @brief Pauses the servo operation.
//  *
//  * This function temporarily stops the servo without fully resetting it.
//  *
//  * @return ESP_OK on success, ESP_FAIL on failure.
//  */
// esp_err_t servo_pause(void);

// /**
//  * @brief Restarts the servo after a pause.
//  *
//  * This function resumes the servo operation from its last paused state.
//  *
//  * @return ESP_OK on success, ESP_FAIL on failure.
//  */
// esp_err_t servo_restart(void);

// /**
//  * @brief Reads the current angle of the servo.
//  *
//  * @return The current servo angle in degrees.
//  */
// int16_t readAngle(void);

// /**
//  * @brief Sets the servo rotation speed based on the given direction.
//  *
//  * @param direction The direction to set (UP or DOWN).
//  */
// void servo_set_speed(SERVO_DIRECTION);

// /**
//  * @brief Deletes any semaphores used for servo control.
//  *
//  * @return ESP_OK on success, ESP_FAIL on failure.
//  */
// void servo_invert(void);

// /**
//  * @brief Deletes any semaphores used for servo control.
//  *
//  * @return ESP_OK on success, ESP_FAIL on failure.
//  */
// esp_err_t delete_servo_semaphores(void);

// #endif  /* _SERVO_H_ */