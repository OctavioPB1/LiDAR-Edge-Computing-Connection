/**
 * @file servo_angle_example.h
 * @brief Header for servo angle reading example
 * 
 * @version 1.0
 * @date 2025-01-27
 * @author Assistant
 */

#ifndef _SERVO_ANGLE_EXAMPLE_H_
#define _SERVO_ANGLE_EXAMPLE_H_

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Start the servo angle reading example task
 * 
 * This function creates a task that demonstrates how to use the
 * readAngle_simple() function to track the angular position of
 * a continuous rotation servo.
 */
void start_servo_angle_example(void);

#ifdef __cplusplus
}
#endif

#endif /* _SERVO_ANGLE_EXAMPLE_H_ */