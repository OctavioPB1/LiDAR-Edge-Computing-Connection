/**
 * @file vl53l0x.h
 * @brief VL53L0X Time-of-Flight ranging sensor control library for ESP32
 * 
 * This library provides functions to initialize and control the VL53L0X sensor on the ESP32.
 * It supports configuration of the device and performing a measurement when tasked to.
 * 
 * @version 2.0
 * @date 2025-02-09
 * 
 * @note
 * - Ensure the 'VCC', 'GND', 'SDA' and 'SCL' pins are connected correctly to the hardware.
 * - Call 'vl53l0x_init' before invoking other functions.
 * - For mapping applications, use continuous mode with interrupt-based reading
 * 
 * @author Artful Bytes / Enhanced for mapping applications
 * @see https://github.com/artfulbytes
 * 
 */

#ifndef VL53L0X_H
#define VL53L0X_H

#include "LiDAR/i2c_vl53l0x.h"
#include "gpio/gpio.h"
#include <stdbool.h>
#include <stdint.h>


#define VL53L0X_OUT_OF_RANGE (8190)

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
 * @brief VL53L0X measurement modes
 */
typedef enum {
    VL53L0X_MODE_SINGLE_SHOT = 0,
    VL53L0X_MODE_CONTINUOUS
} vl53l0x_mode_t;

/**
 * @brief VL53L0X timing budget options (measurement time)
 * 
 * The timing budget determines how long the sensor takes to perform each measurement.
 * Longer timing budgets provide better accuracy but slower measurement rates.
 * 
 * IMPORTANT: The timing budget must be less than or equal to the inter-measurement period
 * configured in the mapping system (measurement_period_ms * 1000).
 * 
 * Trade-offs:
 * - Shorter timing = Higher speed, Lower accuracy, Higher noise
 * - Longer timing = Lower speed, Higher accuracy, Lower noise
 */
typedef enum {
    VL53L0X_TIMING_BUDGET_15MS = 15000,   /**< 15ms timing budget
                                           *   - Speed: ~50Hz measurement rate
                                           *   - Accuracy: Lower (good for fast response)
                                           *   - Use for: Real-time applications, obstacle avoidance
                                           *   - Power: Higher consumption due to fast operation */
    
    VL53L0X_TIMING_BUDGET_20MS = 20000,   /**< 20ms timing budget
                                           *   - Speed: ~40Hz measurement rate
                                           *   - Accuracy: Balanced (good for most applications)
                                           *   - Use for: General mapping, robot navigation
                                           *   - Power: Moderate consumption */
    
    VL53L0X_TIMING_BUDGET_33MS = 33000,   /**< 33ms timing budget
                                           *   - Speed: ~25Hz measurement rate
                                           *   - Accuracy: Better (good for detailed mapping)
                                           *   - Use for: Detailed mapping, precision applications
                                           *   - Power: Lower consumption */
    
    VL53L0X_TIMING_BUDGET_50MS = 50000,   /**< 50ms timing budget
                                           *   - Speed: ~15Hz measurement rate
                                           *   - Accuracy: High (good for quality-critical applications)
                                           *   - Use for: High-precision mapping, calibration
                                           *   - Power: Low consumption */
    
    VL53L0X_TIMING_BUDGET_100MS = 100000, /**< 100ms timing budget
                                           *   - Speed: ~8Hz measurement rate
                                           *   - Accuracy: Maximum (best possible accuracy)
                                           *   - Use for: Maximum precision, static mapping
                                           *   - Power: Lowest consumption */
} vl53l0x_timing_budget_t;

/**
 * @brief VL53L0X measurement data structure
 * 
 * This structure contains all the raw measurement data from the VL53L0X sensor.
 * Use this for advanced applications that need access to raw sensor data.
 */
typedef struct {
    uint16_t range_mm;          /**< Measured distance in millimeters
                                 *   - Valid range: 30-4000mm (depends on sensor configuration)
                                 *   - Returns VL53L0X_OUT_OF_RANGE (500) if measurement failed
                                 *   - This is the raw, unfiltered distance measurement */
    
    uint16_t signal_rate;       /**< Signal rate in MCPS (Mega Counts Per Second)
                                 *   - Indicates the strength of the reflected signal
                                 *   - Higher values = stronger signal = better measurement quality
                                 *   - Typical range: 0-1000 MCPS
                                 *   - Values below 50 MCPS may indicate poor measurement quality
                                 *   - Use this to assess measurement reliability */
    
    uint16_t ambient_rate;      /**< Ambient light rate in MCPS
                                 *   - Indicates the amount of ambient light hitting the sensor
                                 *   - Higher values = more ambient light = potential interference
                                 *   - Typical range: 0-1000 MCPS
                                 *   - High ambient light can reduce measurement accuracy
                                 *   - Use this to detect environmental conditions */
    
    uint8_t range_status;       /**< Range status code
                                 *   - 0: Good measurement
                                 *   - 1: Sigma fail (measurement uncertainty too high)
                                 *   - 2: Signal fail (signal too weak)
                                 *   - 3: Range fail (out of range)
                                 *   - 4: Wraparound fail (ambiguous measurement)
                                 *   - 5: Xtalk fail (crosstalk detected)
                                 *   - 6: Synchronization fail (timing issue)
                                 *   - 7: Min range fail (too close)
                                 *   - 8: Phase fail (phase measurement failed)
                                 *   - 9: Hardware fail (hardware error)
                                 *   - 10: Range valid (good measurement)
                                 *   - 11: Range valid with large pulse (good but bright target)
                                 *   - 12: Range valid with no target (no target detected)
                                 *   - 13: Range valid with large pulse and no target (bright environment) */
    
    uint32_t timestamp_us;      /**< Measurement timestamp in microseconds
                                 *   - Time when this measurement was taken
                                 *   - Use for timing analysis and motion compensation
                                 *   - Can be used to calculate measurement intervals
                                 *   - Useful for debugging and performance analysis */
    
    bool valid;                 /**< Measurement validity flag
                                 *   - true: Measurement is valid and can be used
                                 *   - false: Measurement failed or is unreliable
                                 *   - Check this before using range_mm value
                                 *   - Invalid measurements should be discarded or flagged */
} vl53l0x_measurement_t;

/**
 * @brief Callback function type for interrupt-based readings
 */
typedef void (*vl53l0x_data_ready_cb_t)(vl53l0x_idx_t idx, vl53l0x_measurement_t *measurement);

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
 * @brief Configures the timing budget for measurements
 * @param idx Sensor index
 * @param budget_us Timing budget in microseconds
 * @return ESP_OK on success, ESP_FAIL on failure
 */
esp_err_t vl53l0x_set_timing_budget(vl53l0x_idx_t idx, vl53l0x_timing_budget_t budget_us);

/**
 * @brief Starts continuous ranging mode
 * @param idx Sensor index
 * @param inter_measurement_period_ms Period between measurements in ms
 * @return ESP_OK on success, ESP_FAIL on failure
 */
esp_err_t vl53l0x_start_continuous(vl53l0x_idx_t idx, uint32_t inter_measurement_period_ms);

/**
 * @brief Stops continuous ranging mode
 * @param idx Sensor index
 * @return ESP_OK on success, ESP_FAIL on failure
 */
esp_err_t vl53l0x_stop_continuous(vl53l0x_idx_t idx);

/**
 * @brief Reads measurement data in continuous mode
 * @param idx Sensor index
 * @param measurement Pointer to measurement structure
 * @return ESP_OK on success, ESP_FAIL on failure
 */
esp_err_t vl53l0x_read_range_continuous(vl53l0x_idx_t idx, vl53l0x_measurement_t *measurement);

/**
 * @brief Checks if new measurement data is available
 * @param idx Sensor index
 * @return true if data is ready, false otherwise
 */
bool vl53l0x_is_data_ready(vl53l0x_idx_t idx);

/**
 * @brief Installs interrupt handler for data ready events
 * @param idx Sensor index
 * @param gpio_num GPIO number connected to sensor interrupt pin
 * @param callback Callback function to be called when data is ready
 * @return ESP_OK on success, ESP_FAIL on failure
 */
esp_err_t vl53l0x_install_interrupt_handler(vl53l0x_idx_t idx, gpio_num_t gpio_num, vl53l0x_data_ready_cb_t callback);

/**
 * @brief Removes interrupt handler
 * @param idx Sensor index
 * @return ESP_OK on success, ESP_FAIL on failure
 */
esp_err_t vl53l0x_remove_interrupt_handler(vl53l0x_idx_t idx);

/**
 * @brief Configures signal and ambient rate limits
 * @param idx Sensor index
 * @param signal_rate_limit Signal rate limit in MCPS (Mega Counts Per Second)
 * @param ambient_rate_limit Ambient rate limit in MCPS
 * @return ESP_OK on success, ESP_FAIL on failure
 */
esp_err_t vl53l0x_set_signal_rate_limit(vl53l0x_idx_t idx, float signal_rate_limit, float ambient_rate_limit);

/**
 * @brief Gets current measurement rate capability
 * @param idx Sensor index
 * @return Maximum measurement rate in Hz, 0 on error
 */
float vl53l0x_get_max_measurement_rate(vl53l0x_idx_t idx);

/**
 * Restarts the VL53L0X sensor
 * @return
 * - `ESP_OK`: If the sensor was successfully reset.
 * - `ESP_FAIL`: If the reset fails.
 * @note   Each sensor must have its XSHUT pin connected.
 */
esp_err_t vl53l0x_reset();

#endif /* VL53L0X_H */
