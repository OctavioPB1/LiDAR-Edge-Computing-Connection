/**
 * @file checkpoint.h
 * @author 
 *      Ossola Florencia (flor.ossola13@gmail.com)
 * @brief Checkpoint system for aborting execution in an ESP32 application.
 * 
 * This library was intended to implement a checkpoint mechanism to 
 * abort execution safely, release resources, and reset semaphores. 
 * However, it remains incomplete and is not fully functional.
 * 
 * WARNING: This library is a work in progress and does not currently 
 * ensure a complete rollback or proper system recovery.
 * 
 * @date 2025-02-09
 * @version 0.1 (INCOMPLETE)
 */

#ifndef _CHECKPOINT_H_
#define _CHECKPOINT_H_

#include "esp_err.h"
#include "esp_log.h"
#include "cyclops_core.h"
#include "instruction_buffer.h"
#include "i2c.h"
#include "limit_switch.h"
#include "servo_compatibility.h"

/**
 * @brief Sets a checkpoint for execution rollback.
 * 
 * This function attempts to create a checkpoint using `setjmp()`. 
 * If a rollback is triggered via `longjmp()`, execution will 
 * return to this checkpoint.
 * 
 * @return esp_err_t ESP_OK if the checkpoint is set successfully, 
 * or ESP_FAIL if an error occurs.
 */
esp_err_t setCheckpoint(void);

/**
 * @brief Activates the checkpoint rollback mechanism.
 * 
 * This function is intended to clean up resources, abort tasks, 
 * and reset semaphores before rolling back execution. However, 
 * the implementation is incomplete.
 * 
 * WARNING: This function does not fully abort tasks or clean
 * memory as intended. Additional implementation is required. 
 */
void activeCheckpoint(void);

#endif // _CHECKPOINT_H_