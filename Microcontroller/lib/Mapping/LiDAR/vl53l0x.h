
/**
 * @file vl53l0x.h
 * @brief VL53L0X Time-of-Flight ranging sensor control library for ESP32
 * 
 * This library provides functions to initialize and control the VL53L0X sensor on the ESP32.
 * It supports configuration of the device and performing a measurement when tasked to.
 * 
 * @version 1.0
 * @date 2021-08-25
 * 
 * @note
 * - Ensure the 'VCC', 'GND', 'SDA' and 'SCL' pins are connected correctly to the hardware.
 * - Call 'vl53l0x_init' before invoking other functions.
 * 
 * @author Artful Bytes
 * @see https://github.com/artfulbytes
 * 
 */

#ifndef VL53L0X_H
#define VL53L0X_H

#include "i2c_vl53l0x.h"
#include "../gpio/gpio.h"
#include <stdbool.h>
#include <stdint.h>


#define VL53L0X_OUT_OF_RANGE (500)

/* Comment these out if not connected */
// #define VL53L0X_SECOND
// #define VL53L0X_THIRD

typedef enum
{
    VL53L0X_IDX_FIRST,
#ifdef VL53L0X_SECOND
    VL53L0X_IDX_SECOND,
#endif
#ifdef VL53L0X_THIRD
    VL53L0X_IDX_THIRD,
#endif
} vl53l0x_idx_t;

/**
 * Initializes the sensors in the vl53l0x_idx_t enum.
 * @note Each sensor must have its XSHUT pin connected.
 */
bool vl53l0x_init(void);

/**
 * Does a single range measurement
 * @param idx selects specific sensor
 * @param range contains the measured range or VL53L0X_OUT_OF_RANGE
 *        if out of range.
 * @return
 * - `ESP_OK`: If the distance was successfully read.
 * - `ESP_FAIL`: If the reading fails.
 * @note   Polling-based
 */
esp_err_t vl53l0x_read_range_single(vl53l0x_idx_t idx, uint16_t *range);

/**
 * Restarts the VL53L0X sensor
 * @return
 * - `ESP_OK`: If the sensor was successfully reset.
 * - `ESP_FAIL`: If the reset fails.
 * @note   Each sensor must have its XSHUT pin connected.
 */
esp_err_t vl53l0x_reset();

#endif /* VL53L0X_H */
