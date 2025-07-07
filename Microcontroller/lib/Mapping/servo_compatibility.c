/**
 * @file servo_compatibility_simple.c
 * @brief Simple compatibility layer implementation
 * 
 * This file implements a simple compatibility layer that avoids conflicts
 * with the original servo library by using different function names.
 * 
 * @version 1.0
 * @date 2025-01-27
 * @author Assistant
 */

#include "servo_compatibility.h"
#include "Generic_servo/servo_generic.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include <string.h>

static const char *TAG = "SERVO_SIMPLE";

// Global servo handle for the generic library
static servo_handle_t *g_servo_handle = NULL;

// State tracking for compatibility
static bool g_is_initialized = false;
static bool g_is_enabled = false;
static bool g_is_paused = false;
static uint32_t g_last_speed = SERVO_STOP;
static servo_direction_t g_last_direction = SERVO_DIR_STOP;
static bool g_direction_inverted = false;

// Semaphores for thread safety
static SemaphoreHandle_t g_servo_semaphore = NULL;

// Speed mapping for the old API
static const struct {
    uint32_t pulse_us;
    uint8_t speed;
    servo_direction_t direction;
} SPEED_MAPPING[] = {
    {SERVO_MAX_SPEED_CW,     100, SERVO_DIR_CW},
    {SERVO_MEDIUM_SPEED_CW,   50,  SERVO_DIR_CW},
    {SERVO_LOW_SPEED_CW,      25,  SERVO_DIR_CW},
    {SERVO_STOP,               0,  SERVO_DIR_STOP},
    {SERVO_LOW_SPEED_CCW,     25,  SERVO_DIR_CCW},
    {SERVO_MEDIUM_SPEED_CCW,  50,  SERVO_DIR_CCW},
    {SERVO_MAX_SPEED_CCW,    100,  SERVO_DIR_CCW}
};

static uint8_t pulse_to_speed(uint32_t pulse_us)
{
    for (int i = 0; i < sizeof(SPEED_MAPPING) / sizeof(SPEED_MAPPING[0]); i++) {
        if (SPEED_MAPPING[i].pulse_us == pulse_us) {
            return SPEED_MAPPING[i].speed;
        }
    }
    return 0; // Default to stop
}

static servo_direction_t pulse_to_direction(uint32_t pulse_us)
{
    for (int i = 0; i < sizeof(SPEED_MAPPING) / sizeof(SPEED_MAPPING[0]); i++) {
        if (SPEED_MAPPING[i].pulse_us == pulse_us) {
            return SPEED_MAPPING[i].direction;
        }
    }
    return SERVO_DIR_STOP;
}

esp_err_t servo_simple_initialize(void)
{
    if (g_is_initialized) {
        ESP_LOGW(TAG, "Servo already initialized");
        return ESP_OK;
    }

    // Create semaphore for compatibility
    g_servo_semaphore = xSemaphoreCreateBinary();
    if (!g_servo_semaphore) {
        ESP_LOGE(TAG, "Failed to create semaphore");
        return ESP_FAIL;
    }
    xSemaphoreGive(g_servo_semaphore);

    // Use the predefined continuous servo configuration
    esp_err_t ret = servo_generic_init(&SERVO_CONFIG_CONTINUOUS_STANDARD, &g_servo_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize generic servo: %s", esp_err_to_name(ret));
        vSemaphoreDelete(g_servo_semaphore);
        return ret;
    }

    g_is_initialized = true;
    g_is_enabled = false;
    g_is_paused = false;
    g_last_speed = SERVO_STOP;
    g_last_direction = SERVO_DIR_STOP;

    ESP_LOGI(TAG, "Servo simple compatibility layer initialized successfully");
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
        }
    }

    // Set to medium speed CCW (original behavior)
    if (ret == ESP_OK) {
        uint8_t speed = pulse_to_speed(SERVO_MEDIUM_SPEED_CCW);
        servo_direction_t direction = pulse_to_direction(SERVO_MEDIUM_SPEED_CCW);
        
        ret = servo_generic_set_speed(g_servo_handle, speed, direction);
        if (ret == ESP_OK) {
            g_last_speed = SERVO_MEDIUM_SPEED_CCW;
            g_last_direction = direction;
        }
    }

    xSemaphoreGive(g_servo_semaphore);
    
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Servo started with medium speed CCW");
    }
    
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

    esp_err_t ret = servo_generic_stop(g_servo_handle);
    if (ret == ESP_OK) {
        g_last_speed = SERVO_STOP;
        g_last_direction = SERVO_DIR_STOP;
        ESP_LOGI(TAG, "Servo stopped");
    }

    xSemaphoreGive(g_servo_semaphore);
    return ret;
}

esp_err_t servo_simple_pause(void)
{
    if (!g_is_initialized || !g_servo_handle) {
        ESP_LOGE(TAG, "Servo not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (xSemaphoreTake(g_servo_semaphore, portMAX_DELAY) != pdTRUE) {
        return ESP_FAIL;
    }

    esp_err_t ret = servo_generic_stop(g_servo_handle);
    if (ret == ESP_OK) {
        g_is_paused = true;
        ESP_LOGI(TAG, "Servo paused");
    }

    xSemaphoreGive(g_servo_semaphore);
    return ret;
}

esp_err_t servo_simple_restart(void)
{
    if (!g_is_initialized || !g_servo_handle) {
        ESP_LOGE(TAG, "Servo not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (xSemaphoreTake(g_servo_semaphore, portMAX_DELAY) != pdTRUE) {
        return ESP_FAIL;
    }

    esp_err_t ret = ESP_OK;

    if (g_is_paused) {
        // Restore the last speed and direction
        uint8_t speed = pulse_to_speed(g_last_speed);
        ret = servo_generic_set_speed(g_servo_handle, speed, g_last_direction);
        if (ret == ESP_OK) {
            g_is_paused = false;
            ESP_LOGI(TAG, "Servo restarted");
        }
    }

    xSemaphoreGive(g_servo_semaphore);
    return ret;
}

int16_t readAngle_simple(void)
{
    // For continuous rotation servos, we can't read the actual angle
    ESP_LOGW(TAG, "Angle reading not available for continuous rotation servos");
    return -1;
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

    // Determine current speed level and direction
    uint32_t current_pulse = servo_generic_get_pulse(g_servo_handle);
    servo_direction_t current_direction = pulse_to_direction(current_pulse);

    // Calculate new speed based on direction
    uint32_t new_pulse = current_pulse;
    
    if (direction == SERVO_UP_SIMPLE) {
        // Increase speed
        if (current_direction == SERVO_DIR_CW) {
            if (current_pulse == SERVO_LOW_SPEED_CW) {
                new_pulse = SERVO_MEDIUM_SPEED_CW;
            } else if (current_pulse == SERVO_MEDIUM_SPEED_CW) {
                new_pulse = SERVO_MAX_SPEED_CW;
            }
        } else if (current_direction == SERVO_DIR_CCW) {
            if (current_pulse == SERVO_LOW_SPEED_CCW) {
                new_pulse = SERVO_MEDIUM_SPEED_CCW;
            } else if (current_pulse == SERVO_MEDIUM_SPEED_CCW) {
                new_pulse = SERVO_MAX_SPEED_CCW;
            }
        }
    } else if (direction == SERVO_DOWN_SIMPLE) {
        // Decrease speed
        if (current_direction == SERVO_DIR_CW) {
            if (current_pulse == SERVO_MAX_SPEED_CW) {
                new_pulse = SERVO_MEDIUM_SPEED_CW;
            } else if (current_pulse == SERVO_MEDIUM_SPEED_CW) {
                new_pulse = SERVO_LOW_SPEED_CW;
            }
        } else if (current_direction == SERVO_DIR_CCW) {
            if (current_pulse == SERVO_MAX_SPEED_CCW) {
                new_pulse = SERVO_MEDIUM_SPEED_CCW;
            } else if (current_pulse == SERVO_MEDIUM_SPEED_CCW) {
                new_pulse = SERVO_LOW_SPEED_CCW;
            }
        }
    }

    // Apply the new speed
    if (new_pulse != current_pulse) {
        uint8_t new_speed = pulse_to_speed(new_pulse);
        servo_direction_t new_direction = pulse_to_direction(new_pulse);
        
        esp_err_t ret = servo_generic_set_speed(g_servo_handle, new_speed, new_direction);
        if (ret == ESP_OK) {
            g_last_speed = new_pulse;
            g_last_direction = new_direction;
            ESP_LOGI(TAG, "Speed changed to pulse: %lu, speed: %d, direction: %d", 
                    new_pulse, new_speed, new_direction);
        }
    }

    xSemaphoreGive(g_servo_semaphore);
}

void servo_simple_invert(void)
{
    if (!g_is_initialized || !g_servo_handle) {
        ESP_LOGE(TAG, "Servo not initialized");
        return;
    }

    if (xSemaphoreTake(g_servo_semaphore, portMAX_DELAY) != pdTRUE) {
        return;
    }

    // Toggle direction inversion
    g_direction_inverted = !g_direction_inverted;
    
    // Update the generic servo configuration
    g_servo_handle->config.invert_direction = g_direction_inverted;
    
    // Reapply current speed with new direction
    uint8_t current_speed = pulse_to_speed(g_last_speed);
    servo_direction_t current_direction = g_last_direction;
    
    esp_err_t ret = servo_generic_set_speed(g_servo_handle, current_speed, current_direction);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Servo direction inverted: %s", g_direction_inverted ? "true" : "false");
    }

    xSemaphoreGive(g_servo_semaphore);
}

esp_err_t delete_servo_simple_semaphores(void)
{
    if (g_servo_semaphore) {
        vSemaphoreDelete(g_servo_semaphore);
        g_servo_semaphore = NULL;
    }

    if (g_servo_handle) {
        esp_err_t ret = servo_generic_deinit(g_servo_handle);
        g_servo_handle = NULL;
        g_is_initialized = false;
        g_is_enabled = false;
        g_is_paused = false;
        return ret;
    }

    return ESP_OK;
}