/**
 * @file servo_compatibility_simple.c
 * @brief Simple compatibility layer implementation for SG90
 * 
 * This file implements a simple compatibility layer that avoids conflicts
 * with the original servo library by using different function names.
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

#include "servo_compatibility.h"
#include "Generic_Library/servo_generic.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "esp_timer.h"
#include "debug_helper.h"
#include <string.h>
#include <math.h>

static const char *TAG = "SERVO_SG90";

// Global servo handle for the generic library
static servo_handle_t *g_servo_handle = NULL;

// State tracking for compatibility
static bool g_is_initialized = false;
static bool g_is_enabled = false;
static bool g_is_paused = false;
static uint16_t g_current_position = 90;  // Start at center position
static bool g_direction_inverted = false;

// Continuous sweep state
static bool g_continuous_sweep_active = false;
static bool g_continuous_sweep_paused = false;
static uint32_t g_sweep_duration_ms = 15000; // 15 seconds for full sweep

// Semaphores for thread safety
static SemaphoreHandle_t g_servo_semaphore = NULL;

esp_err_t servo_simple_initialize(void)
{
    if (g_is_initialized) {
        LOG_MESSAGE_W(TAG, "Servo already initialized");
        DEBUGING_ESP_LOG(ESP_LOGW(TAG, "Servo already initialized"));
        return ESP_OK;
    }

    // Create semaphore for thread safety
    g_servo_semaphore = xSemaphoreCreateBinary();
    if (!g_servo_semaphore) {
        LOG_MESSAGE_E(TAG, "Failed to create semaphore");
        DEBUGING_ESP_LOG(ESP_LOGE(TAG, "Failed to create semaphore"));
        return ESP_FAIL;
    }
    xSemaphoreGive(g_servo_semaphore);

    // Initialize the SG90 servo using the configuration from main.c
    esp_err_t ret = servo_generic_init(&SG90_CONFIG, &g_servo_handle);
    if (ret != ESP_OK) {
        LOG_MESSAGE_E(TAG, "Failed to initialize SG90 servo");
        DEBUGING_ESP_LOG(ESP_LOGE(TAG, "Failed to initialize SG90 servo: %s", esp_err_to_name(ret)));
        vSemaphoreDelete(g_servo_semaphore);
        return ret;
    }

    g_is_initialized = true;
    g_is_enabled = false;
    g_is_paused = false;
    g_current_position = SERVO_POSITION_CENTER;
    g_direction_inverted = false;
    g_continuous_sweep_active = false;
    g_continuous_sweep_paused = false;

    LOG_MESSAGE_I(TAG, "SG90 servo compatibility layer initialized successfully");
    DEBUGING_ESP_LOG(ESP_LOGI(TAG, "Pin: %d, Range: %d-%d°, Center: %d°", SG90_CONFIG.pin, SERVO_SG90_MIN_ANGLE, SERVO_SG90_MAX_ANGLE, SERVO_POSITION_CENTER));
    return ESP_OK;
}

esp_err_t servo_simple_start(void)
{
    if (!g_is_initialized || !g_servo_handle) {
        LOG_MESSAGE_E(TAG, "Servo not initialized");
        DEBUGING_ESP_LOG(ESP_LOGE(TAG, "Servo not initialized"));
        return ESP_ERR_INVALID_STATE;
    }

    if (xSemaphoreTake(g_servo_semaphore, portMAX_DELAY) != pdTRUE) {
        LOG_MESSAGE_E(TAG, "Failed to take semaphore");
        DEBUGING_ESP_LOG(ESP_LOGE(TAG, "Failed to take semaphore"));
        return ESP_FAIL;
    }

    esp_err_t ret = ESP_OK;

    // Enable the servo
    if (!g_is_enabled) {
        ret = servo_generic_enable(g_servo_handle);
        if (ret == ESP_OK) {
            g_is_enabled = true;
            LOG_MESSAGE_I(TAG, "SG90 servo enabled");
            DEBUGING_ESP_LOG(ESP_LOGI(TAG, "SG90 servo enabled"));
        }
    }

    // Move to center position
    if (ret == ESP_OK) {
        ret = servo_generic_set_position(g_servo_handle, SERVO_POSITION_MIN);
        if (ret == ESP_OK) {
            g_current_position = SERVO_POSITION_MIN;
            LOG_MESSAGE_I(TAG, "SG90 moved to start position");
            DEBUGING_ESP_LOG(ESP_LOGI(TAG, "SG90 moved to start position (%d°)", SERVO_POSITION_MIN));
        }
    }

    xSemaphoreGive(g_servo_semaphore);
    return ret;
}

esp_err_t servo_simple_stop(void)
{
    if (!g_is_initialized || !g_servo_handle) {
        LOG_MESSAGE_E(TAG, "Servo not initialized");
        DEBUGING_ESP_LOG(ESP_LOGE(TAG, "Servo not initialized"));
        return ESP_ERR_INVALID_STATE;
    }

    if (xSemaphoreTake(g_servo_semaphore, portMAX_DELAY) != pdTRUE) {
        LOG_MESSAGE_E(TAG, "Failed to take semaphore");
        DEBUGING_ESP_LOG(ESP_LOGE(TAG, "Failed to take semaphore"));
        return ESP_FAIL;
    }

    esp_err_t ret = ESP_OK;

    // Stop continuous sweep if active
    if (g_continuous_sweep_active) {
        g_continuous_sweep_active = false;
        g_continuous_sweep_paused = false;
        LOG_MESSAGE_I(TAG, "Continuous sweep stopped");
        DEBUGING_ESP_LOG(ESP_LOGI(TAG, "Continuous sweep stopped"));
    }

    // Move to center position (stop)
    ret = servo_generic_set_position(g_servo_handle, SERVO_POSITION_CENTER);
    if (ret == ESP_OK) {
        g_current_position = SERVO_POSITION_CENTER;
        LOG_MESSAGE_I(TAG, "SG90 stopped at center position");
        DEBUGING_ESP_LOG(ESP_LOGI(TAG, "SG90 stopped at center position (%d°)", SERVO_POSITION_CENTER));
    }

    xSemaphoreGive(g_servo_semaphore);
    return ret;
}

esp_err_t servo_simple_pause(void)
{
    if (!g_is_initialized) {
        LOG_MESSAGE_E(TAG, "Servo not initialized");
        DEBUGING_ESP_LOG(ESP_LOGE(TAG, "Servo not initialized"));
        return ESP_ERR_INVALID_STATE;
    }

    if (xSemaphoreTake(g_servo_semaphore, portMAX_DELAY) != pdTRUE) {
        LOG_MESSAGE_E(TAG, "Failed to take semaphore");
        DEBUGING_ESP_LOG(ESP_LOGE(TAG, "Failed to take semaphore"));
        return ESP_FAIL;
    }

    g_is_paused = true;
    
    // Pause continuous sweep if active
    if (g_continuous_sweep_active) {
        g_continuous_sweep_paused = true;
        LOG_MESSAGE_I(TAG, "Continuous sweep paused");
        DEBUGING_ESP_LOG(ESP_LOGI(TAG, "Continuous sweep paused"));
    }
    
    LOG_MESSAGE_I(TAG, "SG90 servo paused at position");
    DEBUGING_ESP_LOG(ESP_LOGI(TAG, "SG90 servo paused at position (%d°)", g_current_position));

    xSemaphoreGive(g_servo_semaphore);
    return ESP_OK;
}

esp_err_t servo_simple_restart(void)
{
    if (!g_is_initialized) {
        LOG_MESSAGE_E(TAG, "Servo not initialized");
        DEBUGING_ESP_LOG(ESP_LOGE(TAG, "Servo not initialized"));
        return ESP_ERR_INVALID_STATE;
    }

    if (xSemaphoreTake(g_servo_semaphore, portMAX_DELAY) != pdTRUE) {
        LOG_MESSAGE_E(TAG, "Failed to take semaphore");
        DEBUGING_ESP_LOG(ESP_LOGE(TAG, "Failed to take semaphore"));
        return ESP_FAIL;
    }

    g_is_paused = false;
    
    // Resume continuous sweep if it was paused
    if (g_continuous_sweep_active && g_continuous_sweep_paused) {
        g_continuous_sweep_paused = false;
        LOG_MESSAGE_I(TAG, "Continuous sweep resumed");
        DEBUGING_ESP_LOG(ESP_LOGI(TAG, "Continuous sweep resumed"));
    }
    
    LOG_MESSAGE_I(TAG, "SG90 servo restarted");
    DEBUGING_ESP_LOG(ESP_LOGI(TAG, "SG90 servo restarted"));

    xSemaphoreGive(g_servo_semaphore);
    return ESP_OK;
}

int16_t readAngle_simple(void)
{
    if (!g_is_initialized) {
        LOG_MESSAGE_E(TAG, "Servo not initialized");
        DEBUGING_ESP_LOG(ESP_LOGE(TAG, "Servo not initialized"));
        return 0;
    }

    if (xSemaphoreTake(g_servo_semaphore, portMAX_DELAY) != pdTRUE) {
        LOG_MESSAGE_E(TAG, "Failed to take semaphore");
        DEBUGING_ESP_LOG(ESP_LOGE(TAG, "Failed to take semaphore"));
        return 0;
    }
// // Use a short timeout instead of portMAX_DELAY to prevent blocking
// if (xSemaphoreTake(g_servo_semaphore, pdMS_TO_TICKS(10)) != pdTRUE) {
//     // If semaphore is busy, return the last known position
//     DEBUGING_ESP_LOG(ESP_LOGW(TAG, "Servo semaphore busy, returning cached position: %d°", g_current_position));
//     return (int16_t)g_current_position;
// }

    // Get current position from the servo
    uint16_t current_pos = servo_generic_get_position(g_servo_handle);
    g_current_position = current_pos;

    xSemaphoreGive(g_servo_semaphore);
    
    // Return as int16_t for compatibility
    return (int16_t)current_pos;
}

void servo_simple_set_speed(SERVO_DIRECTION_SIMPLE direction)
{
    if (!g_is_initialized || !g_servo_handle) {
        LOG_MESSAGE_E(TAG, "Servo not initialized");
        DEBUGING_ESP_LOG(ESP_LOGE(TAG, "Servo not initialized"));
        return;
    }

    if (xSemaphoreTake(g_servo_semaphore, portMAX_DELAY) != pdTRUE) {
        LOG_MESSAGE_E(TAG, "Failed to take semaphore");
        DEBUGING_ESP_LOG(ESP_LOGE(TAG, "Failed to take semaphore"));
        return;
    }

    uint16_t new_position = g_current_position;
    uint16_t step = 10;
    
    // Calculate new position based on direction
    if (direction == SERVO_UP_SIMPLE) {
        // Move towards higher angle (0->180)
        if (g_current_position < SERVO_POSITION_MAX) {
            new_position = g_current_position + step; // Move 10 degrees
            if (new_position > SERVO_POSITION_MAX) {
                new_position = SERVO_POSITION_MAX;
            }
        }
    } else if (direction == SERVO_DOWN_SIMPLE) {
        // Move towards lower angle (180->0)
        if (g_current_position > step) {
            new_position = g_current_position - step; // Move 10 degrees
        } else {
            new_position = SERVO_POSITION_MIN;
        }
    }

    // Apply direction inversion if needed
    if (g_direction_inverted) {
        if (direction == SERVO_UP_SIMPLE) {
            if (g_current_position > step) {
                new_position = g_current_position - step;
            } else {
                new_position = SERVO_POSITION_MIN;
            }
        } else {
            new_position = g_current_position + step;
            if (new_position > SERVO_POSITION_MAX) {
                new_position = SERVO_POSITION_MAX;
            }
        }
    }

    // Move to new position
    esp_err_t ret = servo_generic_set_position(g_servo_handle, new_position);
    if (ret == ESP_OK) {
        g_current_position = new_position;
        LOG_MESSAGE_I(TAG, "SG90 moved to position");
        DEBUGING_ESP_LOG(ESP_LOGI(TAG, "SG90 moved to position (%d°)", new_position));
    }

    xSemaphoreGive(g_servo_semaphore);
}

void servo_simple_invert(void)
{
    if (!g_is_initialized) {
        LOG_MESSAGE_E(TAG, "Servo not initialized");
        DEBUGING_ESP_LOG(ESP_LOGE(TAG, "Servo not initialized"));
        return;
    }

    if (xSemaphoreTake(g_servo_semaphore, portMAX_DELAY) != pdTRUE) {
        LOG_MESSAGE_E(TAG, "Failed to take semaphore");
        DEBUGING_ESP_LOG(ESP_LOGE(TAG, "Failed to take semaphore"));
        return;
    }

    g_direction_inverted = !g_direction_inverted;
    
    // Invertir inmediatamente la posición actual
    uint16_t inverted_position = SERVO_POSITION_MAX - g_current_position;
    servo_generic_set_position(g_servo_handle, inverted_position);
    g_current_position = inverted_position;
    
    LOG_MESSAGE_I(TAG, "SG90 direction inverted and moved to position");
    DEBUGING_ESP_LOG(ESP_LOGI(TAG, "SG90 direction inverted and moved to position (%d°)", inverted_position));

    xSemaphoreGive(g_servo_semaphore);
}

esp_err_t delete_servo_simple_semaphores(void)
{
    if (g_servo_semaphore) {
        vSemaphoreDelete(g_servo_semaphore);
        g_servo_semaphore = NULL;
        LOG_MESSAGE_I(TAG, "SG90 servo semaphores deleted");
        DEBUGING_ESP_LOG(ESP_LOGI(TAG, "SG90 servo semaphores deleted"));
    }
    return ESP_OK;
}

// Additional functions used by main.c

esp_err_t servo_simple_set_position(uint16_t angle)
{
    if (!g_is_initialized || !g_servo_handle) {
        LOG_MESSAGE_E(TAG, "Servo not initialized");
        DEBUGING_ESP_LOG(ESP_LOGE(TAG, "Servo not initialized"));
        return ESP_ERR_INVALID_STATE;
    }

    if (angle > SERVO_SG90_MAX_ANGLE) {
        LOG_MESSAGE_E(TAG, "Invalid angle");
        DEBUGING_ESP_LOG(ESP_LOGE(TAG, "Invalid angle: %d° (valid range: %d-%d°)", angle, SERVO_SG90_MIN_ANGLE, SERVO_SG90_MAX_ANGLE));
        return ESP_ERR_INVALID_ARG;
    }

    if (xSemaphoreTake(g_servo_semaphore, portMAX_DELAY) != pdTRUE) {
        LOG_MESSAGE_E(TAG, "Failed to take semaphore");
        DEBUGING_ESP_LOG(ESP_LOGE(TAG, "Failed to take semaphore"));
        return ESP_FAIL;
    }

    esp_err_t ret = servo_generic_set_position(g_servo_handle, angle);
    if (ret == ESP_OK) {
        g_current_position = angle;
        LOG_MESSAGE_I(TAG, "SG90 moved to position");
        DEBUGING_ESP_LOG(ESP_LOGI(TAG, "SG90 moved to position (%d°)", angle));
    }

    xSemaphoreGive(g_servo_semaphore);
    return ret;
}

bool servo_simple_is_enabled(void)
{
    if (!g_is_initialized || !g_servo_handle) {
        LOG_MESSAGE_E(TAG, "Servo not initialized");
        DEBUGING_ESP_LOG(ESP_LOGE(TAG, "Servo not initialized"));
        return false;
    }

    if (xSemaphoreTake(g_servo_semaphore, portMAX_DELAY) != pdTRUE) {
        LOG_MESSAGE_E(TAG, "Failed to take semaphore");
        DEBUGING_ESP_LOG(ESP_LOGE(TAG, "Failed to take semaphore"));
        return false;
    }

    bool is_enabled = servo_generic_is_enabled(g_servo_handle);
    
    xSemaphoreGive(g_servo_semaphore);
    return is_enabled;
}

/**
 * @brief Custom smooth movement function that releases semaphore periodically
 * 
 * This function moves the servo smoothly while releasing the semaphore periodically
 * to allow other functions like readAngle() to work during the movement.
 * 
 * @param target_angle Target angle to move to
 * @param duration_ms Total duration of the movement
 * @return esp_err_t ESP_OK on success, error code on failure
 */
static esp_err_t servo_smooth_move_with_semaphore_release(uint16_t target_angle, uint32_t duration_ms)
{
    if (!g_is_initialized || !g_servo_handle) {
        return ESP_ERR_INVALID_STATE;
    }

    uint16_t start_angle = g_current_position;
    uint32_t steps = duration_ms / (1000 / g_servo_handle->config.frequency_hz); // 20ms per step
    
    if (steps == 0) {
        steps = 1;
    }

    float angle_step = (float)(target_angle - start_angle) / steps;
    
    for (uint32_t i = 0; i <= steps; i++) {
        // Take semaphore for this step
        if (xSemaphoreTake(g_servo_semaphore, pdMS_TO_TICKS(100)) != pdTRUE) {
            LOG_MESSAGE_E(TAG, "Failed to take semaphore during smooth movement");
            //return ESP_FAIL;
        }

        // Calculate and set current position
        uint16_t current_angle = start_angle + (uint16_t)(angle_step * i);
        esp_err_t ret = servo_generic_set_position(g_servo_handle, current_angle);
        if (ret == ESP_OK) {
            g_current_position = current_angle;
        }

        // Release semaphore immediately after setting position
        xSemaphoreGive(g_servo_semaphore);
        
        // If there was an error, return it
        if (ret != ESP_OK) {
            return ret;
        }
        
        // Wait for next step (semaphore is released during this time)
        vTaskDelay(pdMS_TO_TICKS(1000 / g_servo_handle->config.frequency_hz));
    }
    
    return ESP_OK;
}

esp_err_t servo_simple_continuous_sweep(void)
{
    esp_err_t ret = ESP_OK;
    
    if (!g_is_initialized || !g_servo_handle) {
        LOG_MESSAGE_E(TAG, "Servo not initialized");
        DEBUGING_ESP_LOG(ESP_LOGE(TAG, "Servo not initialized"));
        return ESP_ERR_INVALID_STATE;
    }

    if (xSemaphoreTake(g_servo_semaphore, portMAX_DELAY) != pdTRUE) {
        LOG_MESSAGE_E(TAG, "Failed to take semaphore");
        DEBUGING_ESP_LOG(ESP_LOGE(TAG, "Failed to take semaphore"));
        return ESP_FAIL;
    }

    // Start continuous sweep if not already active
    if (!g_continuous_sweep_active) {
        g_continuous_sweep_active = true;
        g_continuous_sweep_paused = false;
        LOG_MESSAGE_I(TAG, "Starting continuous sweep (0° <-> 180°)");
        DEBUGING_ESP_LOG(ESP_LOGI(TAG, "Starting continuous sweep (0° <-> 180°)"));
    }

    // Check if paused
    if (g_continuous_sweep_paused) {
        xSemaphoreGive(g_servo_semaphore);
        return ESP_OK;
    }

    uint16_t current_pos = g_current_position;
    uint16_t target_pos;
    
    // Determine target position
    if (current_pos < SERVO_POSITION_MAX) {
        target_pos = SERVO_POSITION_MAX;
        LOG_MESSAGE_I(TAG, "Sweeping to 180°");
    } else {
        target_pos = SERVO_POSITION_MIN;
        LOG_MESSAGE_I(TAG, "Sweeping to 0°");
    }

    xSemaphoreGive(g_servo_semaphore);

    // Use our custom smooth move function that releases semaphore periodically
    ret = servo_smooth_move_with_semaphore_release(target_pos, g_sweep_duration_ms);
    
    if (ret == ESP_OK) {
        LOG_MESSAGE_I(TAG, "Reached target position");
        DEBUGING_ESP_LOG(ESP_LOGI(TAG, "Reached target position (%d°)", target_pos));
    }

    return ret;
}