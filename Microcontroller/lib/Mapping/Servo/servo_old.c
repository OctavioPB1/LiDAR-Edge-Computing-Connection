// /**
//  * @file servo.c
//  * @author Guerrico Leonel (lguerrico@outlook.com)
//  * @brief Implementación del control de un servomotor de giro continuo utilizando ESP32 y MCPWM.
//  * 
//  * Este archivo implementa la inicialización y control de un servomotor de giro continuo mediante
//  * la API de MCPWM del ESP32. Se incluyen funciones para inicializar el servo, ajustar su velocidad,
//  * invertir su dirección y leer el ángulo estimado de rotación.
//  * 
//  * @date 2025-02-09
//  * @version 1.0
//  */
// #include "servo.h"
// #include "driver/mcpwm_prelude.h"
// #include "esp_log.h"
// #include "limit_switch.h"
// #include "esp_timer.h"
// #include "freertos/FreeRTOS.h"
// #include "freertos/semphr.h"
// #include <string.h>
// #include <driver/gpio.h>
// #include "debug_helper.h"


// // Parámetros específicos para el servomotor de giro continuo
// #define SERVO_MIN_PULSEWIDTH_US 900   // Ancho de pulso mínimo (giro rápido en un sentido)
// #define SERVO_MAX_PULSEWIDTH_US 2100  // Ancho de pulso máximo (giro rápido en el sentido opuesto)
// #define SERVO_STOP_PULSEWIDTH_US 1500 // Ancho de pulso para detener el servo

// #define SERVO_PULSE_GPIO GPIO_NUM_14         // GPIO connects to the PWM signal line
// #define SERVO_TIMEBASE_RESOLUTION_HZ 1000000 // 1MHz, 1us per tick
// #define SERVO_TIMEBASE_PERIOD 20000          // 20000 ticks, 20ms

// #define CONVERSION_FACTOR 1000000
// #define BASE_SPEED 545.45
// #define DIFFERENTIAL 600
// #define W_5V 0.00055  // grados/µs

// /** @brief Logging tag for debugging */
// static const char *TAG = "SERVOMOTOR";

// /** @brief MCPWM comparator handle */
// static mcpwm_cmpr_handle_t comparator = NULL;

// /** @brief MCPWM timer handle */
// static mcpwm_timer_handle_t timer = NULL;

// /** @brief Current servo duty cycle (in microseconds) */
// static uint32_t current_duty = SERVO_STOP_PULSEWIDTH_US;

// /** @brief Clockwise (CW) angle of rotation speed*/
// static const int16_t cw_limit = 30;

// /** @brief Counterclockwise (CCW) angle of rotation speed*/
// static const int16_t ccw_limit = -30;

// /** @brief Time base variable for tracking operation time */
// static volatile uint64_t time_base = 0;

// /** @brief Last recorded angle offset */
// static volatile uint16_t last_angle_offset = 0;

// /** @brief Current servo angle */
// static volatile uint16_t angle = 0;

// /** @brief Speed value for the next update */
// static volatile uint32_t next_speed = 0;

// /** @brief Flag indicating a change in speed */
// static volatile bool change_speed_flag = false;

// /** @brief Partial time base for intermediate calculations */
// static volatile uint64_t partialTimeBase = 0;

// /** @brief Pause time base for tracking idle durations */
// static volatile uint64_t pauseTimeBase = 0;

// /** @brief Semaphore for limiting concurrent access to servo parameters */
// static SemaphoreHandle_t limit_semaphore;

// /** @brief Semaphore for protecting current duty cycle modifications */
// static SemaphoreHandle_t current_duty_semaphore;

// /** @brief Semaphore for managing speed change synchronization */
// static SemaphoreHandle_t speed_change_semaphore;

// /**
//  * @brief Sets the servo speed in an ISR-safe manner.
//  * 
//  * @param duty Duty cycle value to set (in microseconds)
//  * @return esp_err_t ESP_OK on success, ESP_FAIL on error.
//  */
// static esp_err_t servo_set_speed_ISR(uint32_t);

// /**
//  * @brief Initializes the servo motor.
//  *
//  * This function configures the necessary MCPWM components (timer, operator, comparator, generator)
//  * and initializes semaphores for thread-safe operations. It also sets up the servo for proper
//  * operation using ESP32 hardware PWM.
//  *
//  * @return esp_err_t ESP_OK on success, ESP_FAIL on failure.
//  */
// esp_err_t servo_initialize(void)
// {

//     // Initialize semaphores
//     limit_semaphore = xSemaphoreCreateBinary();
//     xSemaphoreGive(limit_semaphore);
//     if(limit_semaphore == NULL)
//     {
//         ESP_LOGE(TAG,"ERROR: limit_semaphore is NULL");
//         LOG_MESSAGE_E(TAG,"ERROR: limit_semaphore is NULL");
//         return ESP_FAIL;
//     }
//     current_duty_semaphore = xSemaphoreCreateBinary();
//     xSemaphoreGive(current_duty_semaphore);
//     if(current_duty_semaphore == NULL)
//     {
//         ESP_LOGE(TAG,"ERROR: current_duty_semaphore is NULL");
//         LOG_MESSAGE_E(TAG,"ERROR: current_duty_semaphore is NULL");
//         return ESP_FAIL;
//     }
//     speed_change_semaphore = xSemaphoreCreateBinary();
//     xSemaphoreGive(speed_change_semaphore);
//     if(speed_change_semaphore == NULL)
//     {
//         ESP_LOGE(TAG,"ERROR: speed_change_semaphore is NULL");
//         LOG_MESSAGE_E(TAG,"ERROR: speed_change_semaphore is NULL");
//         return ESP_FAIL;
//     }
    
//     // MCPWM Timer Configuration
//     DEBUGING_ESP_LOG(ESP_LOGI(TAG, "Creating timer and operator..."));
//     mcpwm_timer_config_t timer_config = {
//         .group_id = 0,
//         .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
//         .resolution_hz = SERVO_TIMEBASE_RESOLUTION_HZ,
//         .period_ticks = SERVO_TIMEBASE_PERIOD,
//         .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
//     };

//     if (mcpwm_new_timer(&timer_config, &timer) != ESP_OK)
//     {
//         ESP_LOGE(TAG, "ERROR Creating Timer");
//         LOG_MESSAGE_E(TAG,"ERROR Creating Timer");
//         return ESP_FAIL;
//     }

//     // MCPWM Operator Configuration
//     mcpwm_oper_handle_t oper = NULL;
//     mcpwm_operator_config_t operator_config = {
//         .group_id = 0, // Operator must be in the same group as the timer
//     };

//     if (mcpwm_new_operator(&operator_config, &oper) != ESP_OK)
//     {
//         ESP_LOGE(TAG, "ERROR Creating Operator");
//         LOG_MESSAGE_E(TAG,"ERROR Creating Operator");
//         return ESP_FAIL;
//     }

//     // Connect Operator to Timer
//     DEBUGING_ESP_LOG(ESP_LOGI(TAG, "Connecting timer and operator..."));
//     if (mcpwm_operator_connect_timer(oper, timer) != ESP_OK)
//     {
//         ESP_LOGE(TAG, "ERROR Connecting Timer and Operator");
//         LOG_MESSAGE_E(TAG,"ERROR Connecting Timer and Operator");
//         return ESP_FAIL;
//     }
//     else
//     {
//         ESP_LOGI(TAG, "Timer and Operator connected succesfully");
//     }

//     // MCPWM Comparator Configuration
//     DEBUGING_ESP_LOG(ESP_LOGI(TAG, "Creating comparator and generator from the operator"));
//     mcpwm_comparator_config_t comparator_config = {
//         .flags.update_cmp_on_tez = true,
//     };

//     if (mcpwm_new_comparator(oper, &comparator_config, &comparator) != ESP_OK)
//     {
//         ESP_LOGE(TAG, "ERROR Creating Comparator");
//         LOG_MESSAGE_E(TAG,"ERROR Creating Comparator");
//         return ESP_FAIL;
//     }

//     // MCPWM Generator Configuration
//     mcpwm_gen_handle_t generator = NULL;
//     mcpwm_generator_config_t generator_config = {
//         .gen_gpio_num = SERVO_PULSE_GPIO,
//     };

//     if (mcpwm_new_generator(oper, &generator_config, &generator) != ESP_OK)
//     {
//         ESP_LOGE(TAG, "ERROR Creating Generator");
//         LOG_MESSAGE_E(TAG,"ERROR Creating Generator");
//         return ESP_FAIL;
//     }

//     // Configure Generator Actions
//     DEBUGING_ESP_LOG(ESP_LOGI(TAG, "Set Generator action on Timer and Compare event"));
//     if (mcpwm_generator_set_action_on_timer_event(generator, MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)) != ESP_OK ||
//         mcpwm_generator_set_action_on_compare_event(generator, MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, comparator, MCPWM_GEN_ACTION_LOW)) != ESP_OK)
//     {
//         ESP_LOGE(TAG, "Error al configurar los eventos de generador");
//         LOG_MESSAGE_E(TAG,"Error al configurar los eventos de generador");
//         return ESP_FAIL;
//     }

//     // Enable and Start Timer
//     DEBUGING_ESP_LOG(ESP_LOGI(TAG, "Enabling and start timer..."));
//     if (mcpwm_timer_enable(timer) != ESP_OK)
//     {
//         ESP_LOGE(TAG, "ERROR Enabling Timer");
//         LOG_MESSAGE_E(TAG,"ERROR Enabling Timer");
//         return ESP_FAIL;
//     }

//     if (mcpwm_timer_start_stop(timer, MCPWM_TIMER_START_NO_STOP) != ESP_OK)
//     {
//         ESP_LOGE(TAG, "ERROR Starting Timer");
//         LOG_MESSAGE_E(TAG,"ERROR Starting Timer");
//         return ESP_FAIL;
//     }

//     // Initialize Servo Interrupts
//     DEBUGING_ESP_LOG(ESP_LOGI(TAG, "Initializing Servo Interruption"));
//     if (interrupt_init() != ESP_OK)
//     {
//         ESP_LOGE(TAG, "ERROR Initializing Servo Interruption");
//         LOG_MESSAGE_E(TAG,"ERROR Initializing Servo Interruption");
//         return ESP_FAIL;
//     }

//     DEBUGING_ESP_LOG(ESP_LOGI(TAG, "Servomotor inicializado correctamente"));
//     return ESP_OK;
// }

// /**
//  * @brief Starts the servo motor.
//  *
//  * This function sets the servo motor speed to a predefined medium-speed counterclockwise (CCW) value.
//  *
//  * @return esp_err_t ESP_OK on success, ESP_FAIL on failure.
//  */
// esp_err_t servo_start(void)
// {

//     return servo_set_speed_ISR(SERVO_MEDIUM_SPEED_CCW); // Inicia con el duty en 900us
// }

// /**
//  * @brief Stops the servo motor.
//  *
//  * This function sets the servo motor to a stopped state by setting the duty cycle to a neutral position.
//  *
//  * @return esp_err_t ESP_OK on success, ESP_FAIL on failure.
//  */
// esp_err_t servo_stop(void)
// {
//     return servo_set_speed_ISR(SERVO_STOP); // Detiene el servo con el duty en 1500us
// }

// /**
//  * @brief Adjusts the speed of the servo motor.
//  *
//  * This function sets the PWM duty cycle to control the servo speed. 
//  * It verifies that the duty cycle is within the allowed range, then updates the PWM comparator.
//  * Additionally, it updates the `current_duty` variable in a thread-safe manner using a semaphore.
//  *
//  * @param duty The desired duty cycle in microseconds. Must be within `SERVO_MIN_PULSEWIDTH_US` and `SERVO_MAX_PULSEWIDTH_US`.
//  * @return
//  *      - `ESP_OK` on success.
//  *      - `ESP_ERR_INVALID_ARG` if the duty cycle is out of range.
//  *      - `ESP_FAIL` if the PWM comparator update fails.
//  */
// static esp_err_t servo_set_speed_ISR(uint32_t duty)
// {
//     if (duty < SERVO_MIN_PULSEWIDTH_US || duty > SERVO_MAX_PULSEWIDTH_US)
//     {
//         ESP_LOGE(TAG, "Duty fuera del rango permitido");
//         LOG_MESSAGE_E(TAG,"Duty fuera del rango permitido");
//         return ESP_ERR_INVALID_ARG;
//     }
//     DEBUGING_ESP_LOG(ESP_LOGW(TAG, "DUTY IN RANGE, %lu", duty));

//     if (mcpwm_comparator_set_compare_value(comparator, duty) != ESP_OK)
//     {
//         ESP_LOGE(TAG, "Error al ajustar la velocidad");
//         LOG_MESSAGE_E(TAG,"Error al ajustar la velocidad");
//         return ESP_FAIL;
//     }
//     // SE PUEDE QUITAR AL MODIFICAR LA RUTINA DE INICIO.
//     if (xSemaphoreTakeFromISR(current_duty_semaphore, NULL) == pdTRUE)
//     {
//         current_duty = duty;
//         xSemaphoreGive(current_duty_semaphore);
//     }
//     return ESP_OK;
// }

// /**
//  * @brief Inverts the servo motor's direction.
//  *
//  * This function changes the rotation direction of the servo motor by adjusting the PWM duty cycle.
//  * It checks if the servo is currently stopped and prevents inversion if so.
//  * The function safely updates the duty cycle using semaphores and adjusts timing parameters accordingly.
//  * 
//  * Behavior:
//  * - If the servo is moving clockwise (CW), it switches to the corresponding counterclockwise (CCW) speed.
//  * - If the servo is moving counterclockwise (CCW), it switches to the corresponding clockwise (CW) speed.
//  * - If the servo is stopped (`SERVO_STOP`), an error is logged.
//  * 
//  * It also make a time reference when its called, which is used to calculate the position angle.
//  * And if the flag indicates it, it also change the speed.
//  *
//  * @note Uses `xSemaphoreTake` and `xSemaphoreGive` to ensure safe access to shared variables.
//  */
// void servo_invert()
// {
//     esp_err_t err = ESP_OK;

//     if (xSemaphoreTake(current_duty_semaphore, portMAX_DELAY) == pdTRUE)
//     {
//         if (xSemaphoreTake(speed_change_semaphore, portMAX_DELAY) == pdTRUE)
//         {
//             if (change_speed_flag)
//             {
//                 change_speed_flag = false;
//                 current_duty = next_speed;
//             }
//             xSemaphoreGive(speed_change_semaphore);
//         }
//         if (current_duty == SERVO_STOP)
//         {
//             ESP_LOGW(TAG, "Error: Trying to invert orientation while servo is stopped");
//             LOG_MESSAGE_W(TAG,"Error: Trying to invert orientation while servo is stopped");
//         }
//         else if (current_duty > SERVO_STOP)
//         {
//             if (xSemaphoreTake(limit_semaphore, portMAX_DELAY) == pdTRUE)
//             {
//                 last_angle_offset = ccw_limit;
//                 time_base = esp_timer_get_time();
//                 xSemaphoreGive(limit_semaphore);
//             }
//             switch (current_duty)
//             {
//             case SERVO_LOW_SPEED_CCW:
//                 xSemaphoreGive(current_duty_semaphore);
//                 err = servo_set_speed_ISR(SERVO_LOW_SPEED_CW);
//                 break;
//             case SERVO_MEDIUM_SPEED_CCW:
//                 xSemaphoreGive(current_duty_semaphore);
//                 err = servo_set_speed_ISR(SERVO_MEDIUM_SPEED_CW);
//                 break;
//             case SERVO_MAX_SPEED_CCW:
//                 xSemaphoreGive(current_duty_semaphore);
//                 err = servo_set_speed_ISR(SERVO_MAX_SPEED_CW);
//                 break;
//             default:
//                 xSemaphoreGive(current_duty_semaphore);
//                 break;
//             }
//         }
//         else if (current_duty < SERVO_STOP)
//         {
//             if (xSemaphoreTake(limit_semaphore, portMAX_DELAY) == pdTRUE)
//             {
//                 last_angle_offset = cw_limit;
//                 time_base = esp_timer_get_time();
//                 xSemaphoreGive(limit_semaphore);
//             }
//             switch (current_duty)
//             {
//             case SERVO_LOW_SPEED_CW:
//                 xSemaphoreGive(current_duty_semaphore);
//                 err = servo_set_speed_ISR(SERVO_LOW_SPEED_CCW);
//                 break;
//             case SERVO_MEDIUM_SPEED_CW:
//                 xSemaphoreGive(current_duty_semaphore);
//                 err = servo_set_speed_ISR(SERVO_MEDIUM_SPEED_CCW);
//                 break;
//             case SERVO_MAX_SPEED_CW:
//                 xSemaphoreGive(current_duty_semaphore);
//                 err = servo_set_speed_ISR(SERVO_MAX_SPEED_CCW);
//                 break;
//             default:
//                 xSemaphoreGive(current_duty_semaphore);
//                 break;
//             }
//         }
//     }
//     if (err != ESP_OK)
//     {
//         ESP_LOGE(TAG, "Error: Trying to invert servo orientation, rebooting servo....");
//         LOG_MESSAGE_E(TAG,"Error: Trying to invert servo orientation, rebooting servo....");
//     }
//     DEBUGING_ESP_LOG(ESP_LOGW(TAG, "INVERT END"));
// }

// /**
//  * @brief Reads the current angle of the servo motor.
//  *
//  * This function calculates the current angle of the servo motor based on the elapsed time
//  * and the stored duty cycle. It ensures thread-safe access to shared variables using semaphores.
//  *
//  * The calculation considers:
//  * - The last known angle offset.
//  * - The time since the last reference point.
//  * - The current duty cycle of the servo motor.
//  *
//  * @return 
//  *      - The current angle in degrees (0 to 359).
//  *      - `-1` if there is no valid time reference.
//  */
// int16_t readAngle()
// {
//     uint16_t duty = 0;
//     uint64_t time_now = esp_timer_get_time();
//     int16_t angle_offset = 0;
//     uint64_t time_reference = 0;

//     // Obtener variables necesarias con semáforo
//     if (xSemaphoreTake(limit_semaphore, portMAX_DELAY) == pdTRUE)
//     {
//         angle_offset = last_angle_offset;
//         time_reference = time_base;
//         xSemaphoreGive(limit_semaphore);
//     }

//     if(time_reference == 0){
//         return -1;
//     }

//     if (xSemaphoreTake(current_duty_semaphore, portMAX_DELAY) == pdTRUE)
//     {
//         duty = current_duty;
//         xSemaphoreGive(current_duty_semaphore);
//     }

//     // Calcular velocidad escalada y ángulo
//     int64_t temp = ((int64_t)BASE_SPEED * (duty - SERVO_STOP)) * (time_now - time_reference);
//     int16_t angle = (int16_t)(((temp / (DIFFERENTIAL * CONVERSION_FACTOR)) + angle_offset) % 360);
//     //ESP_LOGE(TAG, "Angle = %" PRIi16, angle);

//     DEBUGING_ESP_LOG(ESP_LOGW(TAG, "Angle = %" PRIi16, angle));

//     return (angle);
// }

// /**
//  * @brief Adjusts the speed of the servo motor based on the given direction.
//  *
//  * This function increases or decreases the servo speed based on the `dir` parameter.
//  * The speed is changed stepwise between predefined speed levels.
//  *
//  * Behavior:
//  * - If `dir == UP`, the speed increases to the next level (Low → Medium → Max).
//  * - If `dir == DOWN`, the speed decreases to the previous level (Max → Medium → Low).
//  * - If the servo is stopped, it remains stopped.
//  *
//  * Thread safety is ensured using semaphores when modifying the duty cycle.
//  * The speed is not changed instanly, but set to change when servo_invert() is called.
//  *
//  * @param dir The desired direction (`UP` to increase speed, `DOWN` to decrease speed).
//  */
// void servo_set_speed(SERVO_DIRECTION dir)
// {
//     static volatile uint32_t duty = 0;
//     if (xSemaphoreTake(current_duty_semaphore, portMAX_DELAY) == pdTRUE)
//     {
//         if (current_duty != SERVO_STOP)
//         {
//             switch (dir)
//             {

//             case UP:
//                 if (current_duty == SERVO_LOW_SPEED_CCW || current_duty == SERVO_LOW_SPEED_CW)
//                 {
//                     if (current_duty == SERVO_LOW_SPEED_CCW)
//                     {
//                         duty = SERVO_MEDIUM_SPEED_CCW;
//                     }
//                     else
//                     {
//                         duty = SERVO_MEDIUM_SPEED_CW;
//                     }
//                 }
//                 else
//                 {
//                     if (current_duty == SERVO_MEDIUM_SPEED_CCW || current_duty == SERVO_MEDIUM_SPEED_CW)
//                     {
//                         if (current_duty == SERVO_MEDIUM_SPEED_CCW)
//                         {
//                             duty = SERVO_MAX_SPEED_CCW;
//                         }
//                         else
//                         {
//                             duty = SERVO_MAX_SPEED_CW;
//                         }
//                     }
//                 }
//                 break;
//             case DOWN:
//                 if (current_duty == SERVO_MAX_SPEED_CCW || current_duty == SERVO_MAX_SPEED_CW)
//                 {
//                     if (current_duty == SERVO_MAX_SPEED_CCW)
//                     {
//                         duty = SERVO_MEDIUM_SPEED_CCW;
//                     }
//                     else
//                     {
//                         duty = SERVO_MEDIUM_SPEED_CW;
//                     }
//                 }
//                 else
//                 {
//                     if (current_duty == SERVO_MEDIUM_SPEED_CCW || current_duty == SERVO_MEDIUM_SPEED_CW)
//                     {
//                         if (current_duty == SERVO_MEDIUM_SPEED_CCW)
//                         {
//                             duty = SERVO_LOW_SPEED_CCW;
//                         }
//                         else
//                         {
//                             duty = SERVO_LOW_SPEED_CW;
//                         }
//                     }
//                 }

//                 break;
//             default:
//                 current_duty = SERVO_STOP;
//                 break;
//             }
//         }
//         DEBUGING_ESP_LOG(ESP_LOGE(TAG,"VELOCIDAD ACTUAL: %lu",current_duty));
//         xSemaphoreGive(current_duty_semaphore);
//     }
//     if (xSemaphoreTake(speed_change_semaphore, portMAX_DELAY) == pdTRUE)
//     {
//         next_speed = duty;
//         change_speed_flag = 1;
//         xSemaphoreGive(speed_change_semaphore);
//     }
// }

// /**
//  * @brief Pauses the servo motor operation.
//  *
//  * This function stops the servo motor and records the current time as a reference
//  * for resuming operation later. It ensures safe access to shared variables using semaphores.
//  *
//  * @return 
//  *      - `ESP_OK` if the servo was successfully paused.
//  *      - `ESP_FAIL` if stopping the servo failed.
//  *      - `ESP_ERR_TIMEOUT` if semaphore acquisition failed.
//  */
// esp_err_t servo_pause(){

//     if(xSemaphoreTake(limit_semaphore,portMAX_DELAY) == pdTRUE){
//         if(servo_stop() != ESP_OK){
//             ESP_LOGE(TAG,"FAIL TO PAUSE SERVO");
//             LOG_MESSAGE_E(TAG,"FAIL TO PAUSE SERVO");
//             xSemaphoreGive(limit_semaphore);
//             return ESP_FAIL;
//         }
//         partialTimeBase = time_base;
//         pauseTimeBase = esp_timer_get_time();
//         xSemaphoreGive(limit_semaphore);
//         return ESP_OK;
//     }
//     return ESP_ERR_TIMEOUT;
// }

// /**
//  * @brief Resumes the servo motor operation after being paused.
//  *
//  * This function restarts the servo motor and adjusts the reference time to maintain
//  * proper timing after resumption. It ensures safe access to shared variables using semaphores.
//  *
//  * @return 
//  *      - `ESP_OK` if the servo was successfully restarted.
//  *      - `ESP_FAIL` if restarting the servo failed.
//  *      - `ESP_ERR_TIMEOUT` if semaphore acquisition failed.
//  */
// esp_err_t servo_restart(){
//     uint64_t auxTime=0;
//     if(xSemaphoreTake(limit_semaphore,portMAX_DELAY) == pdTRUE){
//         auxTime = esp_timer_get_time() - pauseTimeBase;
//         if(servo_start() != ESP_OK){
//             ESP_LOGE(TAG,"FAIL TO RESTART SERVO");
//             LOG_MESSAGE_E(TAG,"FAIL TO RESTART SERVO");
//             xSemaphoreGive(limit_semaphore);
//             return ESP_FAIL;
//         }
//         partialTimeBase -= auxTime;
//         xSemaphoreGive(limit_semaphore);
//     }
//     else 
//     {
//         return ESP_ERR_TIMEOUT;
//     }
//     return ESP_OK;
// }

// /**
//  * @brief Deletes all semaphores related to the servo motor.
//  *
//  * This function releases memory by deleting the semaphores used for controlling
//  * the servo motor. It ensures that semaphores are not deleted if they are `NULL`.
//  *
//  * @return 
//  *      - `ESP_OK` if all semaphores were successfully deleted.
//  *      - `ESP_FAIL` if any of the semaphores were `NULL` before deletion.
//  */
// esp_err_t delete_servo_semaphores()
// {
//     if (limit_semaphore != NULL) {
//         vSemaphoreDelete(limit_semaphore);
//     }
//     else {
//         return ESP_FAIL;
//     }

//     if (current_duty_semaphore != NULL) {
//         vSemaphoreDelete(current_duty_semaphore);
//     }
//     else {
//         return ESP_FAIL;
//     }

//     if (speed_change_semaphore != NULL) {
//         vSemaphoreDelete(speed_change_semaphore);
//     }
//     else {
//         return ESP_FAIL;
//     }

//     return ESP_OK;
// }