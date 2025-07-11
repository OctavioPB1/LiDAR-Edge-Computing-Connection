/**
 * @file servo_generic.c
 * @brief Generic servo motor control library implementation for ESP32
 * 
 * This file implements the generic servo motor control library that supports
 * different types of servo motors including position servos and continuous
 * rotation servos.
 * 
 * @version 2.0
 * @date 2025-01-27
 * @author Assistant
 */

#include "servo_generic.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>
#include <math.h>
#include <inttypes.h>

static const char *TAG = "SERVO_GENERIC";

// Predefined servo configurations
const servo_config_t SERVO_CONFIG_POSITION_STANDARD = {
    .pin = GPIO_NUM_14,
    .type = SERVO_TYPE_POSITION,
    .min_pulse_us = 500,
    .max_pulse_us = 2500,
    .center_pulse_us = 1500,
    .frequency_hz = 50,
    .min_angle = 0,
    .max_angle = 180,
    .speed = 50,
    .invert_direction = false
};

const servo_config_t SERVO_CONFIG_POSITION_MG996R = {
    .pin = GPIO_NUM_14,
    .type = SERVO_TYPE_POSITION,
    .min_pulse_us = 600,
    .max_pulse_us = 2400,
    .center_pulse_us = 1500,
    .frequency_hz = 50,
    .min_angle = 0,
    .max_angle = 180,
    .speed = 50,
    .invert_direction = false
};

const servo_config_t SERVO_CONFIG_POSITION_SG90 = {
    .pin = GPIO_NUM_14,
    .type = SERVO_TYPE_POSITION,
    .min_pulse_us = 500,
    .max_pulse_us = 2500,
    .center_pulse_us = 1500,
    .frequency_hz = 50,
    .min_angle = 0,
    .max_angle = 180,
    .speed = 50,
    .invert_direction = false
};

const servo_config_t SERVO_CONFIG_CONTINUOUS_STANDARD = {
    .pin = GPIO_NUM_14,
    .type = SERVO_TYPE_CONTINUOUS,
    .min_pulse_us = 900,     // MS-R-1.3-9 minimum pulse
    .max_pulse_us = 2100,    // MS-R-1.3-9 maximum pulse
    .center_pulse_us = 1440, // MS-R-1.3-9 center/stop pulse
    .frequency_hz = 50,
    .min_angle = 0,
    .max_angle = 0,
    .speed = 50,             // Default to 50% speed
    .invert_direction = false
};

const servo_config_t SERVO_CONFIG_CONTINUOUS_FUTABA = {
    .pin = GPIO_NUM_14,
    .type = SERVO_TYPE_CONTINUOUS,
    .min_pulse_us = 1000,
    .max_pulse_us = 2000,
    .center_pulse_us = 1500,
    .frequency_hz = 50,
    .min_angle = 0,
    .max_angle = 0,
    .speed = 50,
    .invert_direction = false
};

// Static function declarations
static esp_err_t servo_set_pulse_width(servo_handle_t *handle, uint32_t pulse_width);
static uint32_t angle_to_pulse(servo_handle_t *handle, uint16_t angle);
static uint16_t pulse_to_angle(servo_handle_t *handle, uint32_t pulse);
static uint32_t speed_to_pulse(servo_handle_t *handle, uint8_t speed, servo_direction_t direction);
static esp_err_t validate_config(const servo_config_t *config);
static esp_err_t validate_angle(servo_handle_t *handle, uint16_t angle);
static esp_err_t validate_speed(uint8_t speed);

esp_err_t servo_generic_init(const servo_config_t *config, servo_handle_t **handle)
{
    if (!config || !handle) {
        ESP_LOGE(TAG, "Invalid parameters");
        return ESP_ERR_INVALID_ARG;
    }

    // Validate configuration
    esp_err_t ret = validate_config(config);
    if (ret != ESP_OK) {
        return ret;
    }

    // Allocate servo handle
    *handle = malloc(sizeof(servo_handle_t));
    if (!*handle) {
        ESP_LOGE(TAG, "Failed to allocate servo handle");
        return ESP_ERR_NO_MEM;
    }

    // Initialize handle
    memset(*handle, 0, sizeof(servo_handle_t));
    memcpy(&(*handle)->config, config, sizeof(servo_config_t));
    (*handle)->current_pulse = config->center_pulse_us;
    (*handle)->current_angle = (config->min_angle + config->max_angle) / 2;
    (*handle)->current_speed = config->speed;
    (*handle)->is_initialized = false;
    (*handle)->is_enabled = false;

    // MCPWM Timer Configuration
    mcpwm_timer_config_t timer_config = {
        .group_id = 0,
        .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .resolution_hz = 1000000, // 1MHz resolution
        .period_ticks = 20000,    // 20ms period (50Hz)
        .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
    };

    ret = mcpwm_new_timer(&timer_config, &(*handle)->timer);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create MCPWM timer: %s", esp_err_to_name(ret));
        free(*handle);
        *handle = NULL;
        return ret;
    }

    // MCPWM Operator Configuration
    mcpwm_operator_config_t operator_config = {
        .group_id = 0,
    };

    ret = mcpwm_new_operator(&operator_config, &(*handle)->operator);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create MCPWM operator: %s", esp_err_to_name(ret));
        mcpwm_del_timer((*handle)->timer);
        free(*handle);
        *handle = NULL;
        return ret;
    }

    // Connect operator to timer
    ret = mcpwm_operator_connect_timer((*handle)->operator, (*handle)->timer);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to connect operator to timer: %s", esp_err_to_name(ret));
        mcpwm_del_operator((*handle)->operator);
        mcpwm_del_timer((*handle)->timer);
        free(*handle);
        *handle = NULL;
        return ret;
    }

    // MCPWM Comparator Configuration
    mcpwm_comparator_config_t comparator_config = {
        .flags.update_cmp_on_tez = true,
    };

    ret = mcpwm_new_comparator((*handle)->operator, &comparator_config, &(*handle)->comparator);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create MCPWM comparator: %s", esp_err_to_name(ret));
        mcpwm_del_operator((*handle)->operator);
        mcpwm_del_timer((*handle)->timer);
        free(*handle);
        *handle = NULL;
        return ret;
    }

    // MCPWM Generator Configuration
    mcpwm_generator_config_t generator_config = {
        .gen_gpio_num = config->pin,
    };

    ret = mcpwm_new_generator((*handle)->operator, &generator_config, &(*handle)->generator);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create MCPWM generator: %s", esp_err_to_name(ret));
        mcpwm_del_comparator((*handle)->comparator);
        mcpwm_del_operator((*handle)->operator);
        mcpwm_del_timer((*handle)->timer);
        free(*handle);
        *handle = NULL;
        return ret;
    }

    // Set initial pulse width
    ret = mcpwm_comparator_set_compare_value((*handle)->comparator, (*handle)->current_pulse);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set initial compare value: %s", esp_err_to_name(ret));
        mcpwm_del_generator((*handle)->generator);
        mcpwm_del_comparator((*handle)->comparator);
        mcpwm_del_operator((*handle)->operator);
        mcpwm_del_timer((*handle)->timer);
        free(*handle);
        *handle = NULL;
        return ret;
    }

    // Configure generator actions - CRITICAL: This generates the actual PWM signal
    ret = mcpwm_generator_set_action_on_timer_event((*handle)->generator,
                                                    MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set generator action on timer event: %s", esp_err_to_name(ret));
        mcpwm_del_generator((*handle)->generator);
        mcpwm_del_comparator((*handle)->comparator);
        mcpwm_del_operator((*handle)->operator);
        mcpwm_del_timer((*handle)->timer);
        free(*handle);
        *handle = NULL;
        return ret;
    }

    ret = mcpwm_generator_set_action_on_compare_event((*handle)->generator,
                                                      MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, (*handle)->comparator, MCPWM_GEN_ACTION_LOW));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set generator action on compare event: %s", esp_err_to_name(ret));
        mcpwm_del_generator((*handle)->generator);
        mcpwm_del_comparator((*handle)->comparator);
        mcpwm_del_operator((*handle)->operator);
        mcpwm_del_timer((*handle)->timer);
        free(*handle);
        *handle = NULL;
        return ret;
    }

    (*handle)->is_initialized = true;
    ESP_LOGI(TAG, "Servo initialized successfully on GPIO %d", config->pin);
    
    return ESP_OK;
}

esp_err_t servo_generic_deinit(servo_handle_t *handle)
{
    if (!handle) {
        return ESP_ERR_INVALID_ARG;
    }

    if (handle->is_enabled) {
        servo_generic_disable(handle);
    }

    if (handle->is_initialized) {
        mcpwm_del_generator(handle->generator);
        mcpwm_del_comparator(handle->comparator);
        mcpwm_del_operator(handle->operator);
        mcpwm_del_timer(handle->timer);
    }

    free(handle);
    ESP_LOGI(TAG, "Servo deinitialized");
    
    return ESP_OK;
}

esp_err_t servo_generic_enable(servo_handle_t *handle)
{
    if (!handle || !handle->is_initialized) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t ret = mcpwm_timer_enable(handle->timer);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enable timer: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = mcpwm_timer_start_stop(handle->timer, MCPWM_TIMER_START_NO_STOP);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start timer: %s", esp_err_to_name(ret));
        return ret;
    }

    handle->is_enabled = true;
    ESP_LOGI(TAG, "Servo enabled");
    
    return ESP_OK;
}

esp_err_t servo_generic_disable(servo_handle_t *handle)
{
    if (!handle || !handle->is_initialized) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t ret = mcpwm_timer_start_stop(handle->timer, MCPWM_TIMER_STOP_EMPTY);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to stop timer: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = mcpwm_timer_disable(handle->timer);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to disable timer: %s", esp_err_to_name(ret));
        return ret;
    }

    handle->is_enabled = false;
    ESP_LOGI(TAG, "Servo disabled");
    
    return ESP_OK;
}

esp_err_t servo_generic_set_position(servo_handle_t *handle, uint16_t angle)
{
    if (!handle || !handle->is_initialized) {
        return ESP_ERR_INVALID_ARG;
    }

    if (handle->config.type != SERVO_TYPE_POSITION) {
        ESP_LOGE(TAG, "Cannot set position on non-position servo");
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t ret = validate_angle(handle, angle);
    if (ret != ESP_OK) {
        return ret;
    }

    uint32_t pulse = angle_to_pulse(handle, angle);
    ret = servo_set_pulse_width(handle, pulse);
    if (ret == ESP_OK) {
        handle->current_angle = angle;
        ESP_LOGI(TAG, "Position set to %d degrees", angle);
    }
    
    return ret;
}

esp_err_t servo_generic_set_speed(servo_handle_t *handle, uint8_t speed, servo_direction_t direction)
{
    if (!handle || !handle->is_initialized) {
        return ESP_ERR_INVALID_ARG;
    }

    if (handle->config.type != SERVO_TYPE_CONTINUOUS) {
        ESP_LOGE(TAG, "Cannot set speed on non-continuous servo");
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t ret = validate_speed(speed);
    if (ret != ESP_OK) {
        return ret;
    }

    uint32_t pulse = speed_to_pulse(handle, speed, direction);
    ret = servo_set_pulse_width(handle, pulse);
    if (ret == ESP_OK) {
        handle->current_speed = speed;
        ESP_LOGI(TAG, "Speed set to %d%% in direction %d", speed, direction);
    }
    
    return ret;
}

esp_err_t servo_generic_stop(servo_handle_t *handle)
{
    if (!handle || !handle->is_initialized) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t ret = servo_set_pulse_width(handle, handle->config.center_pulse_us);
    if (ret == ESP_OK) {
        handle->current_speed = 0;
        ESP_LOGI(TAG, "Servo stopped");
    }
    
    return ret;
}

esp_err_t servo_generic_set_pulse(servo_handle_t *handle, uint32_t pulse_width)
{
    if (!handle || !handle->is_initialized) {
        return ESP_ERR_INVALID_ARG;
    }

    if (pulse_width < handle->config.min_pulse_us || pulse_width > handle->config.max_pulse_us) {
                ESP_LOGE(TAG, "Pulse width %" PRIu32 " out of range [%" PRIu32 ", %" PRIu32 "]",
                 pulse_width, handle->config.min_pulse_us, handle->config.max_pulse_us);
        return ESP_ERR_INVALID_ARG;
    }

    return servo_set_pulse_width(handle, pulse_width);
}

uint16_t servo_generic_get_position(servo_handle_t *handle)
{
    if (!handle || !handle->is_initialized) {
        return 0;
    }
    return handle->current_angle;
}

uint8_t servo_generic_get_speed(servo_handle_t *handle)
{
    if (!handle || !handle->is_initialized) {
        return 0;
    }
    return handle->current_speed;
}

uint32_t servo_generic_get_pulse(servo_handle_t *handle)
{
    if (!handle || !handle->is_initialized) {
        return 0;
    }
    return handle->current_pulse;
}

bool servo_generic_is_enabled(servo_handle_t *handle)
{
    if (!handle || !handle->is_initialized) {
        return false;
    }
    return handle->is_enabled;
}

esp_err_t servo_generic_invert_direction(servo_handle_t *handle)
{
    if (!handle || !handle->is_initialized) {
        return ESP_ERR_INVALID_ARG;
    }

    handle->config.invert_direction = !handle->config.invert_direction;
    ESP_LOGI(TAG, "Direction inverted");
    
    return ESP_OK;
}

esp_err_t servo_generic_set_speed_direction(servo_handle_t *handle, uint8_t speed, servo_direction_t direction)
{
    return servo_generic_set_speed(handle, speed, direction);
}

esp_err_t servo_generic_move_smooth(servo_handle_t *handle, uint16_t target_angle, uint32_t duration_ms)
{
    if (!handle || !handle->is_initialized) {
        return ESP_ERR_INVALID_ARG;
    }

    if (handle->config.type != SERVO_TYPE_POSITION) {
        ESP_LOGE(TAG, "Smooth movement only available for position servos");
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t ret = validate_angle(handle, target_angle);
    if (ret != ESP_OK) {
        return ret;
    }

    uint16_t start_angle = handle->current_angle;
    uint32_t steps = duration_ms / 20; // 20ms per step (50Hz)
    
    if (steps == 0) {
        steps = 1;
    }

    float angle_step = (float)(target_angle - start_angle) / steps;
    
    for (uint32_t i = 0; i <= steps; i++) {
        uint16_t current_angle = start_angle + (uint16_t)(angle_step * i);
        ret = servo_generic_set_position(handle, current_angle);
        if (ret != ESP_OK) {
            return ret;
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
    
    ESP_LOGI(TAG, "Smooth movement completed to %d degrees", target_angle);
    return ESP_OK;
}

servo_config_t servo_generic_create_config(gpio_num_t pin, servo_type_t type, 
                                          uint32_t min_pulse, uint32_t max_pulse, uint32_t center_pulse,
                                          uint32_t frequency, uint16_t min_angle, uint16_t max_angle)
{
    servo_config_t config = {
        .pin = pin,
        .type = type,
        .min_pulse_us = min_pulse,
        .max_pulse_us = max_pulse,
        .center_pulse_us = center_pulse,
        .frequency_hz = frequency,
        .min_angle = min_angle,
        .max_angle = max_angle,
        .speed = 50,
        .invert_direction = false
    };
    
    return config;
}

// Static helper functions

static esp_err_t servo_set_pulse_width(servo_handle_t *handle, uint32_t pulse_width)
{
    if (!handle->is_enabled) {
        ESP_LOGW(TAG, "Servo not enabled, enabling first");
        esp_err_t ret = servo_generic_enable(handle);
        if (ret != ESP_OK) {
            return ret;
        }
    }

    esp_err_t ret = mcpwm_comparator_set_compare_value(handle->comparator, pulse_width);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set pulse width: %s", esp_err_to_name(ret));
        return ret;
    }

    handle->current_pulse = pulse_width;
    return ESP_OK;
}

static uint32_t angle_to_pulse(servo_handle_t *handle, uint16_t angle)
{
    uint32_t pulse_range = handle->config.max_pulse_us - handle->config.min_pulse_us;
    uint16_t angle_range = handle->config.max_angle - handle->config.min_angle;
    
    if (angle_range == 0) {
        return handle->config.center_pulse_us;
    }
    
    uint32_t pulse = handle->config.min_pulse_us + 
                     (uint32_t)((angle - handle->config.min_angle) * pulse_range / angle_range);
    
    return pulse;
}

static uint16_t pulse_to_angle(servo_handle_t *handle, uint32_t pulse)
{
    uint32_t pulse_range = handle->config.max_pulse_us - handle->config.min_pulse_us;
    uint16_t angle_range = handle->config.max_angle - handle->config.min_angle;
    
    if (pulse_range == 0 || angle_range == 0) {
        return handle->config.min_angle;
    }
    
    uint16_t angle = handle->config.min_angle + 
                     (uint16_t)((pulse - handle->config.min_pulse_us) * angle_range / pulse_range);
    
    return angle;
}

static uint32_t speed_to_pulse(servo_handle_t *handle, uint8_t speed, servo_direction_t direction)
{
    if (direction == SERVO_DIR_STOP || speed == 0) {
        return handle->config.center_pulse_us;
    }

    uint32_t pulse_range = handle->config.max_pulse_us - handle->config.min_pulse_us;
    uint32_t half_range = pulse_range / 2;
    uint32_t center = handle->config.center_pulse_us;
    
    // Calculate speed factor (0-100 to 0-1)
    float speed_factor = (float)speed / 100.0f;
    
    uint32_t pulse;
    if (direction == SERVO_DIR_CW) {
        pulse = center - (uint32_t)(half_range * speed_factor);
    } else { // SERVO_DIR_CCW
        pulse = center + (uint32_t)(half_range * speed_factor);
    }
    
    // Apply direction inversion if configured
    if (handle->config.invert_direction) {
        if (direction == SERVO_DIR_CW) {
            pulse = center + (uint32_t)(half_range * speed_factor);
        } else {
            pulse = center - (uint32_t)(half_range * speed_factor);
        }
    }
    
    return pulse;
}

static esp_err_t validate_config(const servo_config_t *config)
{
    if (!config) {
        return ESP_ERR_INVALID_ARG;
    }

    if (config->min_pulse_us >= config->max_pulse_us) {
        ESP_LOGE(TAG, "Invalid pulse range: min >= max");
        return ESP_ERR_INVALID_ARG;
    }

    if (config->center_pulse_us < config->min_pulse_us || 
        config->center_pulse_us > config->max_pulse_us) {
        ESP_LOGE(TAG, "Center pulse outside valid range");
        return ESP_ERR_INVALID_ARG;
    }

    if (config->frequency_hz == 0) {
        ESP_LOGE(TAG, "Invalid frequency");
        return ESP_ERR_INVALID_ARG;
    }

    if (config->type == SERVO_TYPE_POSITION) {
        if (config->min_angle >= config->max_angle) {
            ESP_LOGE(TAG, "Invalid angle range for position servo");
            return ESP_ERR_INVALID_ARG;
        }
    }

    return ESP_OK;
}

static esp_err_t validate_angle(servo_handle_t *handle, uint16_t angle)
{
    if (angle < handle->config.min_angle || angle > handle->config.max_angle) {
        ESP_LOGE(TAG, "Angle %d out of range [%d, %d]", 
                 angle, handle->config.min_angle, handle->config.max_angle);
        return ESP_ERR_INVALID_ARG;
    }
    return ESP_OK;
}

static esp_err_t validate_speed(uint8_t speed)
{
    if (speed > 100) {
        ESP_LOGE(TAG, "Speed %d out of range [0, 100]", speed);
        return ESP_ERR_INVALID_ARG;
    }
    return ESP_OK;
} 