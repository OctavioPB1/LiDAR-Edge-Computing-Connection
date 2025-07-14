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
static uint32_t g_sweep_duration_ms = 3000; // 3 seconds for full sweep

// Semaphores for thread safety
static SemaphoreHandle_t g_servo_semaphore = NULL;

esp_err_t servo_simple_initialize(void)
{
    if (g_is_initialized) {
        ESP_LOGW(TAG, "Servo already initialized");
        return ESP_OK;
    }

    // Create semaphore for thread safety
    g_servo_semaphore = xSemaphoreCreateBinary();
    if (!g_servo_semaphore) {
        ESP_LOGE(TAG, "Failed to create semaphore");
        return ESP_FAIL;
    }
    xSemaphoreGive(g_servo_semaphore);

    // Initialize the SG90 servo using the configuration from main.c
    esp_err_t ret = servo_generic_init(&SG90_CONFIG, &g_servo_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize SG90 servo: %s", esp_err_to_name(ret));
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

    ESP_LOGI(TAG, "SG90 servo compatibility layer initialized successfully");
    ESP_LOGI(TAG, "Pin: %d, Range: %d-%d°, Center: %d°", 
             SG90_CONFIG.pin, SERVO_SG90_MIN_ANGLE, SERVO_SG90_MAX_ANGLE, SERVO_POSITION_CENTER);
    return ESP_OK;
}

esp_err_t servo_simple_start(void)
{
    if (!g_is_initialized || !g_servo_handle) {
        ESP_LOGE(TAG, "Servo not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (xSemaphoreTake(g_servo_semaphore, portMAX_DELAY) != pdTRUE) {
        return ESP_FAIL;
    }

    esp_err_t ret = ESP_OK;

    // Enable the servo
    if (!g_is_enabled) {
        ret = servo_generic_enable(g_servo_handle);
        if (ret == ESP_OK) {
            g_is_enabled = true;
            ESP_LOGI(TAG, "SG90 servo enabled");
        }
    }

    // Move to center position
    if (ret == ESP_OK) {
        ret = servo_generic_set_position(g_servo_handle, SERVO_POSITION_MIN);
        if (ret == ESP_OK) {
            g_current_position = SERVO_POSITION_MIN;
            ESP_LOGI(TAG, "SG90 moved to center position (%d°)", SERVO_POSITION_MIN);
        }
    }

    xSemaphoreGive(g_servo_semaphore);
    return ret;
}

esp_err_t servo_simple_stop(void)
{
    if (!g_is_initialized || !g_servo_handle) {
        ESP_LOGE(TAG, "Servo not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (xSemaphoreTake(g_servo_semaphore, portMAX_DELAY) != pdTRUE) {
        return ESP_FAIL;
    }

    esp_err_t ret = ESP_OK;

    // Stop continuous sweep if active
    if (g_continuous_sweep_active) {
        g_continuous_sweep_active = false;
        g_continuous_sweep_paused = false;
        ESP_LOGI(TAG, "Continuous sweep stopped");
    }

    // Move to center position (stop)
    ret = servo_generic_set_position(g_servo_handle, SERVO_POSITION_CENTER);
    if (ret == ESP_OK) {
        g_current_position = SERVO_POSITION_CENTER;
        ESP_LOGI(TAG, "SG90 stopped at center position (%d°)", SERVO_POSITION_CENTER);
    }

    xSemaphoreGive(g_servo_semaphore);
    return ret;
}

esp_err_t servo_simple_pause(void)
{
    if (!g_is_initialized) {
        ESP_LOGE(TAG, "Servo not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (xSemaphoreTake(g_servo_semaphore, portMAX_DELAY) != pdTRUE) {
        return ESP_FAIL;
    }

    g_is_paused = true;
    
    // Pause continuous sweep if active
    if (g_continuous_sweep_active) {
        g_continuous_sweep_paused = true;
        ESP_LOGI(TAG, "Continuous sweep paused");
    }
    
    ESP_LOGI(TAG, "SG90 servo paused at position %d°", g_current_position);

    xSemaphoreGive(g_servo_semaphore);
    return ESP_OK;
}

esp_err_t servo_simple_restart(void)
{
    if (!g_is_initialized) {
        ESP_LOGE(TAG, "Servo not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (xSemaphoreTake(g_servo_semaphore, portMAX_DELAY) != pdTRUE) {
        return ESP_FAIL;
    }

    g_is_paused = false;
    
    // Resume continuous sweep if it was paused
    if (g_continuous_sweep_active && g_continuous_sweep_paused) {
        g_continuous_sweep_paused = false;
        ESP_LOGI(TAG, "Continuous sweep resumed");
    }
    
    ESP_LOGI(TAG, "SG90 servo restarted");

    xSemaphoreGive(g_servo_semaphore);
    return ESP_OK;
}

int16_t readAngle_simple(void)
{
    if (!g_is_initialized) {
        ESP_LOGE(TAG, "Servo not initialized");
        return 0;
    }

    if (xSemaphoreTake(g_servo_semaphore, portMAX_DELAY) != pdTRUE) {
        return 0;
    }

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
        ESP_LOGE(TAG, "Servo not initialized");
        return;
    }

    if (xSemaphoreTake(g_servo_semaphore, portMAX_DELAY) != pdTRUE) {
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
        ESP_LOGI(TAG, "SG90 moved to position %d° (direction: %s)", 
                 new_position, (direction == SERVO_UP_SIMPLE) ? "UP" : "DOWN");
    }

    xSemaphoreGive(g_servo_semaphore);
}

void servo_simple_invert(void)
{
    if (!g_is_initialized) {
        ESP_LOGE(TAG, "Servo not initialized");
        return;
    }

    if (xSemaphoreTake(g_servo_semaphore, portMAX_DELAY) != pdTRUE) {
        return;
    }

    g_direction_inverted = !g_direction_inverted;
    
    // Invertir inmediatamente la posición actual
    uint16_t inverted_position = SERVO_POSITION_MAX - g_current_position;
    servo_generic_set_position(g_servo_handle, inverted_position);
    g_current_position = inverted_position;
    
    ESP_LOGI(TAG, "SG90 direction inverted and moved to %d°", inverted_position);

    xSemaphoreGive(g_servo_semaphore);
}

esp_err_t delete_servo_simple_semaphores(void)
{
    if (g_servo_semaphore) {
        vSemaphoreDelete(g_servo_semaphore);
        g_servo_semaphore = NULL;
        ESP_LOGI(TAG, "SG90 servo semaphores deleted");
    }
    return ESP_OK;
}

// Additional functions used by main.c

esp_err_t servo_simple_set_position(uint16_t angle)
{
    if (!g_is_initialized || !g_servo_handle) {
        ESP_LOGE(TAG, "Servo not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (angle > SERVO_SG90_MAX_ANGLE) {
        ESP_LOGE(TAG, "Invalid angle: %d° (valid range: %d-%d°)", 
                 angle, SERVO_SG90_MIN_ANGLE, SERVO_SG90_MAX_ANGLE);
        return ESP_ERR_INVALID_ARG;
    }

    if (xSemaphoreTake(g_servo_semaphore, portMAX_DELAY) != pdTRUE) {
        return ESP_FAIL;
    }

    esp_err_t ret = servo_generic_set_position(g_servo_handle, angle);
    if (ret == ESP_OK) {
        g_current_position = angle;
        ESP_LOGI(TAG, "SG90 moved to position %d°", angle);
    }

    xSemaphoreGive(g_servo_semaphore);
    return ret;
}

bool servo_simple_is_enabled(void)
{
    if (!g_is_initialized || !g_servo_handle) {
        return false;
    }

    if (xSemaphoreTake(g_servo_semaphore, portMAX_DELAY) != pdTRUE) {
        return false;
    }

    bool is_enabled = servo_generic_is_enabled(g_servo_handle);
    
    xSemaphoreGive(g_servo_semaphore);
    return is_enabled;
}

esp_err_t servo_simple_continuous_sweep(void)
{
    esp_err_t ret = ESP_OK;
    
    if (!g_is_initialized || !g_servo_handle) {
        ESP_LOGE(TAG, "Servo not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (xSemaphoreTake(g_servo_semaphore, portMAX_DELAY) != pdTRUE) {
        return ESP_FAIL;
    }

    // Start continuous sweep if not already active
    if (!g_continuous_sweep_active) {
        g_continuous_sweep_active = true;
        g_continuous_sweep_paused = false;
        ESP_LOGI(TAG, "Starting continuous sweep (0° <-> 180°)");
    }

    // Perform sweep if not paused
    if (!g_continuous_sweep_paused) {
        // Si está en posición < 180°, ir a 180°
        if (g_current_position < SERVO_POSITION_MAX) {
            ESP_LOGI(TAG, "Sweeping to 180° (smooth movement)");
            ret = servo_generic_move_smooth(g_servo_handle, SERVO_POSITION_MAX, g_sweep_duration_ms);
            if (ret == ESP_OK) {
                g_current_position = SERVO_POSITION_MAX;
                ESP_LOGI(TAG, "Reached 180°");
            }
        } else {
            // Si está en 180°, ir a 0°
            ESP_LOGI(TAG, "Sweeping to 0° (smooth movement)");
            ret = servo_generic_move_smooth(g_servo_handle, SERVO_POSITION_MIN, g_sweep_duration_ms);
            if (ret == ESP_OK) {
                g_current_position = SERVO_POSITION_MIN;
                ESP_LOGI(TAG, "Reached 0°");
            }
        }
    }

    xSemaphoreGive(g_servo_semaphore);
    return ret;
}