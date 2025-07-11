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
#include "esp_timer.h"
#include <string.h>
#include <math.h>

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

// Angle tracking for continuous rotation servos
static float g_current_angle = 0.0f;  // Current angle in degrees
static uint32_t g_last_update_time = 0;  // Last update time in milliseconds
static bool g_angle_tracking_enabled = false;

// Semaphores for thread safety
static SemaphoreHandle_t g_servo_semaphore = NULL;

// MS-R-1.3-9 Speed calculation function
// Linear interpolation: pulse_us -> degrees_per_second
// At 1500us (center): 0 degrees/second
// At 900us (min): -545 degrees/second (CW)
// At 2100us (max): +545 degrees/second (CCW)
static float pulse_to_degrees_per_second(uint32_t pulse_us)
{
    if (pulse_us == SERVO_MS_R_1_3_9_CENTER_PULSE_US) {
        return 0.0f; // Stop
    }
    
    // Calculate the deviation from center
    int32_t deviation = (int32_t)pulse_us - (int32_t)SERVO_MS_R_1_3_9_CENTER_PULSE_US;
    
    // Calculate speed based on linear interpolation
    float speed_ratio = (float)deviation / (float)(SERVO_MS_R_1_3_9_MAX_PULSE_US - SERVO_MS_R_1_3_9_CENTER_PULSE_US);
    
    // Apply maximum speed
    float degrees_per_second = speed_ratio * SERVO_MS_R_1_3_9_MAX_SPEED_DPS;
    
    return degrees_per_second;
}

// Speed mapping for the old API - Updated with MS-R-1.3-9 specifications
static const struct {
    uint32_t pulse_us;
    uint8_t speed;
    servo_direction_t direction;
} SPEED_MAPPING[] = {
    {SERVO_MAX_SPEED_CW,     100, SERVO_DIR_CW},    // 900us  -> -200°/s
    {SERVO_MEDIUM_SPEED_CW,   50, SERVO_DIR_CW},    // 1200us -> -100°/s
    {SERVO_LOW_SPEED_CW,      15, SERVO_DIR_CW},    // 1400us -> -30°/s (slower)
    {SERVO_STOP,               0, SERVO_DIR_STOP},  // 1440us -> 0°/s
    {SERVO_LOW_SPEED_CCW,     15, SERVO_DIR_CCW},   // 1480us -> +30°/s (slower)
    {SERVO_MEDIUM_SPEED_CCW,  50, SERVO_DIR_CCW},   // 1800us -> +100°/s
    {SERVO_MAX_SPEED_CCW,    100, SERVO_DIR_CCW}    // 2100us -> +200°/s
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

static void update_angle_tracking(void)
{
    if (!g_angle_tracking_enabled) {
        return;
    }

    uint32_t current_time = esp_timer_get_time() / 1000; // Convert to milliseconds
    uint32_t elapsed_time = current_time - g_last_update_time;
    
    if (elapsed_time == 0) {
        return;
    }

    // Get angular velocity directly from mapping table
    float angular_velocity = pulse_to_degrees_per_second(g_last_speed);
    
    // Update angle based on elapsed time
    float angle_change = (angular_velocity * elapsed_time) / 1000.0f; // Convert to degrees
    g_current_angle += angle_change;
    
    // Normalize angle to 0-360 range
    while (g_current_angle >= 360.0f) {
        g_current_angle -= 360.0f;
    }
    while (g_current_angle < 0.0f) {
        g_current_angle += 360.0f;
    }
    
    g_last_update_time = current_time;
    
    // Debug logging (remove in production)
    ESP_LOGD(TAG, "Angle tracking: elapsed=%lums, angular_vel=%.2f°/s, angle_change=%.2f°, current_angle=%.2f°", 
             elapsed_time, angular_velocity, angle_change, g_current_angle);
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
    
    // Initialize angle tracking
    g_current_angle = 0.0f;
    g_last_update_time = esp_timer_get_time() / 1000;
    g_angle_tracking_enabled = true;

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

    // Set to low speed CCW (slower than original behavior)
    if (ret == ESP_OK) {
        uint8_t speed = pulse_to_speed(SERVO_LOW_SPEED_CCW);
        servo_direction_t direction = pulse_to_direction(SERVO_LOW_SPEED_CCW);
        
        ret = servo_generic_set_speed(g_servo_handle, speed, direction);
        if (ret == ESP_OK) {
            g_last_speed = SERVO_LOW_SPEED_CCW;
            g_last_direction = direction;
            g_last_update_time = esp_timer_get_time() / 1000; // Reset timer for angle tracking
        }
    }

    xSemaphoreGive(g_servo_semaphore);
    
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Servo started with low speed CCW (slower)");
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
        update_angle_tracking(); // Update angle before stopping
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
        update_angle_tracking(); // Update angle before pausing
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
            g_last_update_time = esp_timer_get_time() / 1000; // Reset timer for angle tracking
            ESP_LOGI(TAG, "Servo restarted");
        }
    }

    xSemaphoreGive(g_servo_semaphore);
    return ret;
}

int16_t readAngle_simple(void)
{
    if (!g_is_initialized) {
        ESP_LOGE(TAG, "Servo not initialized");
        return -1;
    }

    // Update angle tracking to get current position
    update_angle_tracking();
    
    // Return the tracked angle as an integer (0-359 degrees)
    int16_t angle = (int16_t)g_current_angle;
    
    ESP_LOGD(TAG, "Current angle: %d degrees", angle);
    return angle;
}

void servo_simple_reset_angle(void)
{
    if (!g_is_initialized) {
        ESP_LOGE(TAG, "Servo not initialized");
        return;
    }

    if (xSemaphoreTake(g_servo_semaphore, portMAX_DELAY) != pdTRUE) {
        return;
    }

    g_current_angle = 0.0f;
    g_last_update_time = esp_timer_get_time() / 1000;
    ESP_LOGI(TAG, "Angle reset to 0 degrees");

    xSemaphoreGive(g_servo_semaphore);
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
        update_angle_tracking(); // Update angle before changing speed
        
        uint8_t new_speed = pulse_to_speed(new_pulse);
        servo_direction_t new_direction = pulse_to_direction(new_pulse);
        
        esp_err_t ret = servo_generic_set_speed(g_servo_handle, new_speed, new_direction);
        if (ret == ESP_OK) {
            g_last_speed = new_pulse;
            g_last_direction = new_direction;
            g_last_update_time = esp_timer_get_time() / 1000; // Reset timer for angle tracking
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
    update_angle_tracking(); // Update angle before inverting
    
    uint8_t current_speed = pulse_to_speed(g_last_speed);
    servo_direction_t current_direction = g_last_direction;
    
    esp_err_t ret = servo_generic_set_speed(g_servo_handle, current_speed, current_direction);
    if (ret == ESP_OK) {
        g_last_update_time = esp_timer_get_time() / 1000; // Reset timer for angle tracking
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
        
        // Reset angle tracking
        g_current_angle = 0.0f;
        g_last_update_time = 0;
        g_angle_tracking_enabled = false;
        
        return ret;
    }

    return ESP_OK;
}

// Additional calibration and debugging functions
void servo_simple_calibrate_speed(uint32_t pulse_us, float measured_degrees_per_second)
{
    float calculated_speed = pulse_to_degrees_per_second(pulse_us);
    float error_percentage = ((calculated_speed - measured_degrees_per_second) / measured_degrees_per_second) * 100.0f;
    
    ESP_LOGI(TAG, "=== SERVO CALIBRATION RESULTS ===");
    ESP_LOGI(TAG, "Pulse: %lu us", pulse_us);
    ESP_LOGI(TAG, "Calculated speed: %.2f degrees/second", calculated_speed);
    ESP_LOGI(TAG, "Measured speed: %.2f degrees/second", measured_degrees_per_second);
    ESP_LOGI(TAG, "Error: %.2f%%", error_percentage);
    
    if (fabs(error_percentage) > 10.0f) {
        ESP_LOGW(TAG, "Large error detected! Consider adjusting SERVO_MS_R_1_3_9_MAX_SPEED_DPS");
        float suggested_max_speed = SERVO_MS_R_1_3_9_MAX_SPEED_DPS * (measured_degrees_per_second / calculated_speed);
        ESP_LOGI(TAG, "Suggested SERVO_MS_R_1_3_9_MAX_SPEED_DPS: %.2f", suggested_max_speed);
    } else {
        ESP_LOGI(TAG, "Calibration looks good!");
    }
    ESP_LOGI(TAG, "================================");
}

void servo_simple_debug_angle_tracking(void)
{
    if (!g_is_initialized) {
        ESP_LOGE(TAG, "Servo not initialized");
        return;
    }

    ESP_LOGI(TAG, "=== ANGLE TRACKING DEBUG (MS-R-1.3-9) ===");
    ESP_LOGI(TAG, "Current angle: %.2f degrees", g_current_angle);
    ESP_LOGI(TAG, "Last update time: %lu ms", g_last_update_time);
    ESP_LOGI(TAG, "Tracking enabled: %s", g_angle_tracking_enabled ? "YES" : "NO");
    ESP_LOGI(TAG, "Current pulse: %lu us", g_last_speed);
    ESP_LOGI(TAG, "Current direction: %d", g_last_direction);
    ESP_LOGI(TAG, "Angular velocity: %.2f degrees/second", pulse_to_degrees_per_second(g_last_speed));
    ESP_LOGI(TAG, "Servo specifications:");
    ESP_LOGI(TAG, "  - Min pulse: %d us", SERVO_MS_R_1_3_9_MIN_PULSE_US);
    ESP_LOGI(TAG, "  - Max pulse: %d us", SERVO_MS_R_1_3_9_MAX_PULSE_US);
    ESP_LOGI(TAG, "  - Center pulse: %d us", SERVO_MS_R_1_3_9_CENTER_PULSE_US);
    ESP_LOGI(TAG, "  - Max speed: %.2f degrees/second", SERVO_MS_R_1_3_9_MAX_SPEED_DPS);
    ESP_LOGI(TAG, "  - Operating voltage: %.1fV", SERVO_MS_R_1_3_9_VOLTAGE);
    ESP_LOGI(TAG, "=========================================");
}

int servo_simple_is_moving(void)
{
    if (!g_is_initialized) {
        return 0; // false
    }
    
    return (g_last_direction != SERVO_DIR_STOP && g_last_speed != SERVO_STOP) ? 1 : 0;
}

float servo_simple_get_angular_velocity(void)
{
    if (!g_is_initialized) {
        return 0.0f;
    }
    
    return pulse_to_degrees_per_second(g_last_speed);
}