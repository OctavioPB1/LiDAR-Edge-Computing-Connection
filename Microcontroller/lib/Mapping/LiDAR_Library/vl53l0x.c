/**
 * @file vl53l0x.c
 * @brief Implementation of the VL53L0X Time-of-Flight ranging sensor control library for ESP32
 * 
 * This file implements the functions for initializing and controlling the VL53L0X sensor. 
 * It includes basic I2C operations to read the device ID, configure the sensor, and perform
 * on-demand range measurements.
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

#include "vl53l0x.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "debug_helper.h"
#include "esp_timer.h"
#include "driver/gpio.h"
#include "freertos/queue.h"
#include <string.h>


#define REG_IDENTIFICATION_MODEL_ID (0xC0)
#define REG_VHV_CONFIG_PAD_SCL_SDA_EXTSUP_HV (0x89)
#define REG_MSRC_CONFIG_CONTROL (0x60)
#define REG_FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT (0x44)
#define REG_SYSTEM_SEQUENCE_CONFIG (0x01)
#define REG_DYNAMIC_SPAD_REF_EN_START_OFFSET (0x4F)
#define REG_DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD (0x4E)
#define REG_GLOBAL_CONFIG_REF_EN_START_SELECT (0xB6)
#define REG_SYSTEM_INTERRUPT_CONFIG_GPIO (0x0A)
#define REG_GPIO_HV_MUX_ACTIVE_HIGH (0x84)
#define REG_SYSTEM_INTERRUPT_CLEAR (0x0B)
#define REG_RESULT_INTERRUPT_STATUS (0x13)
#define REG_SYSRANGE_START (0x00)
#define REG_GLOBAL_CONFIG_SPAD_ENABLES_REF_0 (0xB0)
#define REG_RESULT_RANGE_STATUS (0x14)
#define REG_SLAVE_DEVICE_ADDRESS (0x8A)

#define RANGE_SEQUENCE_STEP_TCC (0x10) /* Target CentreCheck */
#define RANGE_SEQUENCE_STEP_MSRC (0x04) /* Minimum Signal Rate Check */
#define RANGE_SEQUENCE_STEP_DSS (0x28) /* Dynamic SPAD selection */
#define RANGE_SEQUENCE_STEP_PRE_RANGE (0x40)
#define RANGE_SEQUENCE_STEP_FINAL_RANGE (0x80)

#define VL53L0X_EXPECTED_DEVICE_ID (0xEE)
#define VL53L0X_DEFAULT_ADDRESS (0x29)

/* There are two types of SPAD: aperture and non-aperture. My understanding
 * is that aperture ones let it less light (they have a smaller opening), similar
 * to how you can change the aperture on a digital camera. Only 1/4 th of the
 * SPADs are of type non-aperture. */
#define SPAD_TYPE_APERTURE (0x01)
/* The total SPAD array is 16x16, but we can only activate a quadrant spanning 44 SPADs at
 * a time. In the ST api code they have (for some reason) selected 0xB4 (180) as a starting
 * point (lies in the middle and spans non-aperture (3rd) quadrant and aperture (4th) quadrant). */
#define SPAD_START_SELECT (0xB4)
/* The total SPAD map is 16x16, but we should only activate an area of 44 SPADs at a time. */
#define SPAD_MAX_COUNT (44)
/* The 44 SPADs are represented as 6 bytes where each bit represents a single SPAD.
 * 6x8 = 48, so the last four bits are unused. */
#define SPAD_MAP_ROW_COUNT (6)
#define SPAD_ROW_SIZE (8)
/* Since we start at 0xB4 (180), there are four quadrants (three aperture, one aperture),
 * and each quadrant contains 256 / 4 = 64 SPADs, and the third quadrant is non-aperture, the
 * offset to the aperture quadrant is (256 - 64 - 180) = 12 */
#define SPAD_APERTURE_START_INDEX (12)

static const char *TAG = "VL53L0X";

typedef struct vl53l0x_info
{
    uint8_t addr;
    gpio_t xshut_gpio;
} vl53l0x_info_t;

typedef enum
{
    CALIBRATION_TYPE_VHV,
    CALIBRATION_TYPE_PHASE
} calibration_type_t;

static const vl53l0x_info_t vl53l0x_infos[] =
{
    [VL53L0X_IDX_FIRST] = { .addr = 0x29, .xshut_gpio = GPIO_XSHUT_FIRST },
#ifdef VL53L0X_SECOND
    [VL53L0X_IDX_SECOND] = { .addr = 0x31, .xshut_gpio = GPIO_XSHUT_SECOND },
#endif
#ifdef VL53L0X_THIRD
    [VL53L0X_IDX_THIRD] = { .addr = 0x32, .xshut_gpio = GPIO_XSHUT_THIRD },
#endif
};

static uint8_t stop_variable = 0;

/**
 * We can read the model id to confirm that the device is booted.
 * (There is no fresh_out_of_reset as on the vl6180x)
 */
static bool device_is_booted()
{
    uint8_t device_id = 0;
    if (!i2c_read_addr8_data8(REG_IDENTIFICATION_MODEL_ID, &device_id)) {
        ESP_LOGE(TAG, "Error reading the MODEL ID");
        LOG_MESSAGE_E(TAG,"Error reading the MODEL ID");
        return false;
    }
    DEBUGING_ESP_LOG(ESP_LOGI(TAG, "REG_IDENTIFICATION_MODEL_ID read: %x", device_id));
    return device_id == VL53L0X_EXPECTED_DEVICE_ID;
}

/**
 * One time device initialization
 */
static bool data_init()
{
    bool success = false;

    /* Set 2v8 mode */
    uint8_t vhv_config_scl_sda = 0;
    if (!i2c_read_addr8_data8(REG_VHV_CONFIG_PAD_SCL_SDA_EXTSUP_HV, &vhv_config_scl_sda)) {
        return false;
    }
    vhv_config_scl_sda |= 0x01;
    if (!i2c_write_addr8_data8(REG_VHV_CONFIG_PAD_SCL_SDA_EXTSUP_HV, vhv_config_scl_sda)) {
        return false;
    }

    /* Set I2C standard mode */
    success = i2c_write_addr8_data8(0x88, 0x00);

    success &= i2c_write_addr8_data8(0x80, 0x01);
    success &= i2c_write_addr8_data8(0xFF, 0x01);
    success &= i2c_write_addr8_data8(0x00, 0x00);
    /* It may be unnecessary to retrieve the stop variable for each sensor */
    success &= i2c_read_addr8_data8(0x91, &stop_variable);
    success &= i2c_write_addr8_data8(0x00, 0x01);
    success &= i2c_write_addr8_data8(0xFF, 0x00);
    success &= i2c_write_addr8_data8(0x80, 0x00);

    return success;
}

/**
 * Wait for strobe value to be set. This is used when we read values
 * from NVM (non volatile memory).
 */
static bool read_strobe()
{
    bool success = false;
    uint8_t strobe = 0;
    if (!i2c_write_addr8_data8(0x83, 0x00)) {
        return false;
    }
    do {
        success = i2c_read_addr8_data8(0x83, &strobe);
    } while (success && (strobe == 0));
    if (!success) {
        return false;
    }
    if (!i2c_write_addr8_data8(0x83, 0x01)) {
        return false;
    }
    return true;
}

/**
 * Gets the spad count, spad type och "good" spad map stored by ST in NVM at
 * their production line.
 * .
 * According to the datasheet, ST runs a calibration (without cover glass) and
 * saves a "good" SPAD map to NVM (non volatile memory). The SPAD array has two
 * types of SPADs: aperture and non-aperture. By default, all of the
 * good SPADs are enabled, but we should only enable a subset of them to get
 * an optimized signal rate. We should also only enable either only aperture
 * or only non-aperture SPADs. The number of SPADs to enable and which type
 * are also saved during the calibration step at ST factory and can be retrieved
 * from NVM.
 */
static bool get_spad_info_from_nvm(uint8_t *spad_count, uint8_t *spad_type, uint8_t good_spad_map[6])
{
    bool success = false;
    uint8_t tmp_data8 = 0;
    uint32_t tmp_data32 = 0;

    /* Setup to read from NVM */
    success  = i2c_write_addr8_data8(0x80, 0x01);
    success &= i2c_write_addr8_data8(0xFF, 0x01);
    success &= i2c_write_addr8_data8(0x00, 0x00);
    success &= i2c_write_addr8_data8(0xFF, 0x06);
    success &= i2c_read_addr8_data8(0x83, &tmp_data8);
    success &= i2c_write_addr8_data8(0x83, tmp_data8 | 0x04);
    success &= i2c_write_addr8_data8(0xFF, 0x07);
    success &= i2c_write_addr8_data8(0x81, 0x01);
    success &= i2c_write_addr8_data8(0x80, 0x01);
    if (!success) {
      return false;
    }

    /* Get the SPAD count and type */
    success &= i2c_write_addr8_data8(0x94, 0x6b);
    if (!success) {
        return false;
    }
    if (!read_strobe()) {
        return false;
    }
    success &= i2c_read_addr8_data32(0x90, &tmp_data32);
    if (!success) {
        return false;
    }
    *spad_count = (tmp_data32 >> 8) & 0x7f;
    *spad_type = (tmp_data32 >> 15) & 0x01;

    /* Since the good SPAD map is already stored in REG_GLOBAL_CONFIG_SPAD_ENABLES_REF_0
     * we can simply read that register instead of doing the below */
#if 0
    /* Get the first part of the SPAD map */
    if (!i2c_write_addr8_data8(0x94, 0x24)) {
        return false;
    }
    if (!read_strobe()) {
        return false;
    }
    if (!i2c_read_addr8_data32(0x90, &tmp_data32)) {
      return false;
    }
    good_spad_map[0] = (uint8_t)((tmp_data32 >> 24) & 0xFF);
    good_spad_map[1] = (uint8_t)((tmp_data32 >> 16) & 0xFF);
    good_spad_map[2] = (uint8_t)((tmp_data32 >> 8) & 0xFF);
    good_spad_map[3] = (uint8_t)(tmp_data32 & 0xFF);

    /* Get the second part of the SPAD map */
    if (!i2c_write_addr8_data8(0x94, 0x25)) {
        return false;
    }
    if (!read_strobe()) {
        return false;
    }
    if (!i2c_read_addr8_data32(0x90, &tmp_data32)) {
        return false;
    }
    good_spad_map[4] = (uint8_t)((tmp_data32 >> 24) & 0xFF);
    good_spad_map[5] = (uint8_t)((tmp_data32 >> 16) & 0xFF);

#endif

    /* Restore after reading from NVM */
    success &=i2c_write_addr8_data8(0x81, 0x00);
    success &=i2c_write_addr8_data8(0xFF, 0x06);
    success &=i2c_read_addr8_data8(0x83, &tmp_data8);
    success &=i2c_write_addr8_data8(0x83, tmp_data8 & 0xfb);
    success &=i2c_write_addr8_data8(0xFF, 0x01);
    success &=i2c_write_addr8_data8(0x00, 0x01);
    success &=i2c_write_addr8_data8(0xFF, 0x00);
    success &=i2c_write_addr8_data8(0x80, 0x00);

    /* When we haven't configured the SPAD map yet, the SPAD map register actually
     * contains the good SPAD map, so we can retrieve it straight from this register
     * instead of reading it from the NVM. */
    if (!i2c_read_addr8_bytes(REG_GLOBAL_CONFIG_SPAD_ENABLES_REF_0, good_spad_map, 6)) {
        return false;
    }
    return success;
}

/**
 * Sets the SPADs according to the value saved to NVM by ST during production. Assuming
 * similar conditions (e.g. no cover glass), this should give reasonable readings and we
 * can avoid running ref spad management (tedious code).
 */
static bool set_spads_from_nvm()
{
    uint8_t spad_map[SPAD_MAP_ROW_COUNT] = { 0 };
    uint8_t good_spad_map[SPAD_MAP_ROW_COUNT] = { 0 };
    uint8_t spads_enabled_count = 0;
    uint8_t spads_to_enable_count = 0;
    uint8_t spad_type = 0;
    volatile uint32_t total_val = 0;

    if (!get_spad_info_from_nvm(&spads_to_enable_count, &spad_type, good_spad_map)) {
        return false;
    }

    for (int i = 0; i < 6; i++) {
        total_val += good_spad_map[i];
    }

    bool success = i2c_write_addr8_data8(0xFF, 0x01);
    success &= i2c_write_addr8_data8(REG_DYNAMIC_SPAD_REF_EN_START_OFFSET, 0x00);
    success &= i2c_write_addr8_data8(REG_DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD, 0x2C);
    success &= i2c_write_addr8_data8(0xFF, 0x00);
    success &= i2c_write_addr8_data8(REG_GLOBAL_CONFIG_REF_EN_START_SELECT, SPAD_START_SELECT);
    if (!success) {
        return false;
    }

    uint8_t offset = (spad_type == SPAD_TYPE_APERTURE) ? SPAD_APERTURE_START_INDEX : 0;

    /* Create a new SPAD array by selecting a subset of the SPADs suggested by the good SPAD map.
     * The subset should only have the number of type enabled as suggested by the reading from
     * the NVM (spads_to_enable_count and spad_type). */
    for (int row = 0; row < SPAD_MAP_ROW_COUNT; row++) {
        for (int column = 0; column < SPAD_ROW_SIZE; column++) {
            int index = (row * SPAD_ROW_SIZE) + column;
            if (index >= SPAD_MAX_COUNT) {
                return false;
            }
            if (spads_enabled_count == spads_to_enable_count) {
                /* We are done */
                break;
            }
            if (index < offset) {
                continue;
            }
            if ((good_spad_map[row] >> column) & 0x1) {
                spad_map[row] |= (1 << column);
                spads_enabled_count++;
            }
        }
        if (spads_enabled_count == spads_to_enable_count) {
            /* To avoid looping unnecessarily when we are already done. */
            break;
        }
    }

    if (spads_enabled_count != spads_to_enable_count) {
        return false;
    }

    /* Write the new SPAD configuration */
    if (!i2c_write_addr8_bytes(REG_GLOBAL_CONFIG_SPAD_ENABLES_REF_0, spad_map, SPAD_MAP_ROW_COUNT)) {
        return false;
    }

    return true;
}

/**
 * Load tuning settings (same as default tuning settings provided by ST api code)
 */
static bool load_default_tuning_settings()
{
    bool success = i2c_write_addr8_data8(0xFF, 0x01);
    success &= i2c_write_addr8_data8(0x00, 0x00);
    success &= i2c_write_addr8_data8(0xFF, 0x00);
    success &= i2c_write_addr8_data8(0x09, 0x00);
    success &= i2c_write_addr8_data8(0x10, 0x00);
    success &= i2c_write_addr8_data8(0x11, 0x00);
    success &= i2c_write_addr8_data8(0x24, 0x01);
    success &= i2c_write_addr8_data8(0x25, 0xFF);
    success &= i2c_write_addr8_data8(0x75, 0x00);
    success &= i2c_write_addr8_data8(0xFF, 0x01);
    success &= i2c_write_addr8_data8(0x4E, 0x2C);
    success &= i2c_write_addr8_data8(0x48, 0x00);
    success &= i2c_write_addr8_data8(0x30, 0x20);
    success &= i2c_write_addr8_data8(0xFF, 0x00);
    success &= i2c_write_addr8_data8(0x30, 0x09);
    success &= i2c_write_addr8_data8(0x54, 0x00);
    success &= i2c_write_addr8_data8(0x31, 0x04);
    success &= i2c_write_addr8_data8(0x32, 0x03);
    success &= i2c_write_addr8_data8(0x40, 0x83);
    success &= i2c_write_addr8_data8(0x46, 0x25);
    success &= i2c_write_addr8_data8(0x60, 0x00);
    success &= i2c_write_addr8_data8(0x27, 0x00);
    success &= i2c_write_addr8_data8(0x50, 0x06);
    success &= i2c_write_addr8_data8(0x51, 0x00);
    success &= i2c_write_addr8_data8(0x52, 0x96);
    success &= i2c_write_addr8_data8(0x56, 0x08);
    success &= i2c_write_addr8_data8(0x57, 0x30);
    success &= i2c_write_addr8_data8(0x61, 0x00);
    success &= i2c_write_addr8_data8(0x62, 0x00);
    success &= i2c_write_addr8_data8(0x64, 0x00);
    success &= i2c_write_addr8_data8(0x65, 0x00);
    success &= i2c_write_addr8_data8(0x66, 0xA0);
    success &= i2c_write_addr8_data8(0xFF, 0x01);
    success &= i2c_write_addr8_data8(0x22, 0x32);
    success &= i2c_write_addr8_data8(0x47, 0x14);
    success &= i2c_write_addr8_data8(0x49, 0xFF);
    success &= i2c_write_addr8_data8(0x4A, 0x00);
    success &= i2c_write_addr8_data8(0xFF, 0x00);
    success &= i2c_write_addr8_data8(0x7A, 0x0A);
    success &= i2c_write_addr8_data8(0x7B, 0x00);
    success &= i2c_write_addr8_data8(0x78, 0x21);
    success &= i2c_write_addr8_data8(0xFF, 0x01);
    success &= i2c_write_addr8_data8(0x23, 0x34);
    success &= i2c_write_addr8_data8(0x42, 0x00);
    success &= i2c_write_addr8_data8(0x44, 0xFF);
    success &= i2c_write_addr8_data8(0x45, 0x26);
    success &= i2c_write_addr8_data8(0x46, 0x05);
    success &= i2c_write_addr8_data8(0x40, 0x40);
    success &= i2c_write_addr8_data8(0x0E, 0x06);
    success &= i2c_write_addr8_data8(0x20, 0x1A);
    success &= i2c_write_addr8_data8(0x43, 0x40);
    success &= i2c_write_addr8_data8(0xFF, 0x00);
    success &= i2c_write_addr8_data8(0x34, 0x03);
    success &= i2c_write_addr8_data8(0x35, 0x44);
    success &= i2c_write_addr8_data8(0xFF, 0x01);
    success &= i2c_write_addr8_data8(0x31, 0x04);
    success &= i2c_write_addr8_data8(0x4B, 0x09);
    success &= i2c_write_addr8_data8(0x4C, 0x05);
    success &= i2c_write_addr8_data8(0x4D, 0x04);
    success &= i2c_write_addr8_data8(0xFF, 0x00);
    success &= i2c_write_addr8_data8(0x44, 0x00);
    success &= i2c_write_addr8_data8(0x45, 0x20);
    success &= i2c_write_addr8_data8(0x47, 0x08);
    success &= i2c_write_addr8_data8(0x48, 0x28);
    success &= i2c_write_addr8_data8(0x67, 0x00);
    success &= i2c_write_addr8_data8(0x70, 0x04);
    success &= i2c_write_addr8_data8(0x71, 0x01);
    success &= i2c_write_addr8_data8(0x72, 0xFE);
    success &= i2c_write_addr8_data8(0x76, 0x00);
    success &= i2c_write_addr8_data8(0x77, 0x00);
    success &= i2c_write_addr8_data8(0xFF, 0x01);
    success &= i2c_write_addr8_data8(0x0D, 0x01);
    success &= i2c_write_addr8_data8(0xFF, 0x00);
    success &= i2c_write_addr8_data8(0x80, 0x01);
    success &= i2c_write_addr8_data8(0x01, 0xF8);
    success &= i2c_write_addr8_data8(0xFF, 0x01);
    success &= i2c_write_addr8_data8(0x8E, 0x01);
    success &= i2c_write_addr8_data8(0x00, 0x01);
    success &= i2c_write_addr8_data8(0xFF, 0x00);
    success &= i2c_write_addr8_data8(0x80, 0x00);
    return success;
}

static bool configure_interrupt()
{
    /* Interrupt on new sample ready */
    if (!i2c_write_addr8_data8(REG_SYSTEM_INTERRUPT_CONFIG_GPIO, 0x04)) {
        return false;
    }

    /* Configure active low since the pin is pulled-up on most breakout boards */
    uint8_t gpio_hv_mux_active_high = 0;
    if (!i2c_read_addr8_data8(REG_GPIO_HV_MUX_ACTIVE_HIGH, &gpio_hv_mux_active_high)) {
        return false;
    }
    gpio_hv_mux_active_high &= ~0x10;
    if (!i2c_write_addr8_data8(REG_GPIO_HV_MUX_ACTIVE_HIGH, gpio_hv_mux_active_high)) {
        return false;
    }

    if (!i2c_write_addr8_data8(REG_SYSTEM_INTERRUPT_CLEAR, 0x01)) {
        return false;
    }
    return true;
}


/**
 * Enable (or disable) specific steps in the sequence
 */
static bool set_sequence_steps_enabled(uint8_t sequence_step)
{
    return i2c_write_addr8_data8(REG_SYSTEM_SEQUENCE_CONFIG, sequence_step);
}

/**
 * Basic device initialization
 */
static bool static_init()
{
    if (!set_spads_from_nvm()) {
        return false;
    }

    if (!load_default_tuning_settings()) {
        return false;
    }

    if (!configure_interrupt()) {
        return false;
    }

    if (!set_sequence_steps_enabled(RANGE_SEQUENCE_STEP_DSS +
                                    RANGE_SEQUENCE_STEP_PRE_RANGE +
                                    RANGE_SEQUENCE_STEP_FINAL_RANGE)) {
        return false;
    }

    return true;
}

static bool perform_single_ref_calibration(calibration_type_t calib_type)
{
    uint8_t sysrange_start = 0;
    uint8_t sequence_config = 0;
    switch (calib_type)
    {
    case CALIBRATION_TYPE_VHV:
        sequence_config = 0x01;
        sysrange_start = 0x01 | 0x40;
        break;
    case CALIBRATION_TYPE_PHASE:
        sequence_config = 0x02;
        sysrange_start = 0x01 | 0x00;
        break;
    }
    if (!i2c_write_addr8_data8(REG_SYSTEM_SEQUENCE_CONFIG, sequence_config)) {
        return false;
    }
    if (!i2c_write_addr8_data8(REG_SYSRANGE_START, sysrange_start)) {
        return false;
    }
    /* Wait for interrupt */
    uint8_t interrupt_status = 0;
    bool success = false;
    do {
        success = i2c_read_addr8_data8(REG_RESULT_INTERRUPT_STATUS, &interrupt_status);
    } while (success && ((interrupt_status & 0x07) == 0));
    if (!success) {
        return false;
    }
    if (!i2c_write_addr8_data8(REG_SYSTEM_INTERRUPT_CLEAR, 0x01)) {
        return false;
    }

    if (!i2c_write_addr8_data8(REG_SYSRANGE_START, 0x00)) {
        return false;
    }
    return true;
}

/**
 * Temperature calibration needs to be run again if the temperature changes by
 * more than 8 degrees according to the datasheet.
 */
static bool perform_ref_calibration()
{
    if (!perform_single_ref_calibration(CALIBRATION_TYPE_VHV)) {
        return false;
    }
    if (!perform_single_ref_calibration(CALIBRATION_TYPE_PHASE)) {
        return false;
    }
    /* Restore sequence steps enabled */
    if (!set_sequence_steps_enabled(RANGE_SEQUENCE_STEP_DSS +
                                    RANGE_SEQUENCE_STEP_PRE_RANGE +
                                    RANGE_SEQUENCE_STEP_FINAL_RANGE)) {
        return false;
    }
    return true;
}

static bool configure_address(uint8_t addr)
{
    /* 7-bit address */
    return i2c_write_addr8_data8(REG_SLAVE_DEVICE_ADDRESS, addr & 0x7F);
}

/**
 * Sets the sensor in hardware standby by flipping the XSHUT pin.
 */
static void set_hardware_standby(vl53l0x_idx_t idx, bool enable)
{
    esp_err_t err = gpio_set_output(vl53l0x_infos[idx].xshut_gpio, !enable);
    if (err != ESP_OK){
        ESP_LOGE(TAG, "Error en el set_hardware_standby: %s", esp_err_to_name(err));
        LOG_MESSAGE_E(TAG,"Error en el set_hardware_standby");
    }
}

/**
 * Configures the GPIOs used for the XSHUT pin.
 * Output low by default means the sensors will be in
 * hardware standby after this function is called.
 *
 * NOTE: The pins are hard-coded to P1.0, P1.1, and P1.2.
 **/
static void configure_gpio()
{
    esp_err_t err = gpio_init();
    if (err != ESP_OK){
        ESP_LOGE(TAG, "Fallo en gpio_init(): %s", esp_err_to_name(err));
        LOG_MESSAGE_E(TAG,"Fallo en gpio_init()");
    }

    err = gpio_set_output(GPIO_XSHUT_FIRST, false);
    if (err != ESP_OK){
        ESP_LOGE(TAG, "Fallo en gpio_set_output(): %s", esp_err_to_name(err));
        LOG_MESSAGE_E(TAG,"Fallo en gpio_set_output()");
    }

    // err = gpio_set_output(GPIO_XSHUT_SECOND, false);
    // if (err != ESP_OK){
    //     ESP_LOGI(TAG, "Fallo en configure_gpio(): %s", esp_err_to_name(err));
    // }

    // err = gpio_set_output(GPIO_XSHUT_THIRD, false);
    // if (err != ESP_OK){
    //     ESP_LOGI(TAG, "Fallo en configure_gpio(): %s", esp_err_to_name(err));
    // }
}

/* Sets the address of a single VL53L0X sensor.
 * This functions assumes that all non-configured VL53L0X are still
 * in hardware standby. */
static bool init_address(vl53l0x_idx_t idx)
{
    set_hardware_standby(idx, false);
    //i2c_set_slave_address(VL53L0X_DEFAULT_ADDRESS);

    /* The datasheet doesn't say how long we must wait to leave hw standby,
     * but using the same delay as vl6180x seems to work fine. */
    //__delay_cycles(400);
    vTaskDelay(800/portTICK_PERIOD_MS);

    if (!device_is_booted()) {
        ESP_LOGE(TAG, "El dispisitivo NO está conectado");
        LOG_MESSAGE_E(TAG,"El dispisitivo NO está conectado");
        return false;
    }

    if (!configure_address(vl53l0x_infos[idx].addr)) {
        ESP_LOGE(TAG, "No se ha podido asignar la dirección correctamente: %d", vl53l0x_infos[idx].addr);
        LOG_MESSAGE_E(TAG,"No se ha podido asignar la dirección correctamente");
        return false;
    }
    return true;
}

/**
 * Initializes the sensors by putting them in hw standby and then
 * waking them up one-by-one as described in AN4846.
 */
static bool init_addresses()
{
    /* Puts all sensors in hardware standby */
    configure_gpio();

    /* Wake each sensor up one by one and set a unique address for each one */
    if (!init_address(VL53L0X_IDX_FIRST)) {
        ESP_LOGE(TAG, "Fallo en init_address");
        LOG_MESSAGE_E(TAG,"Fallo en init_address");
        return false;
    }
#ifdef VL53L0X_SECOND
    if (!init_address(VL53L0X_IDX_SECOND)) {
        return false;
    }
#endif
#ifdef VL53L0X_THIRD
    if (!init_address(VL53L0X_IDX_THIRD)) {
        return false;
    }
#endif

    return true;
}

static bool init_config(vl53l0x_idx_t idx)
{
    //i2c_set_slave_address(vl53l0x_infos[idx].addr);
    if (!data_init()) {
        ESP_LOGE(TAG, "Fallo en data_init");
        LOG_MESSAGE_E(TAG,"Fallo en data_init");
        return false;
    }
    if (!static_init()) {
        ESP_LOGE(TAG, "Fallo en static_init");
        LOG_MESSAGE_E(TAG,"Fallo en static_init");
        return false;
    }
    if (!perform_ref_calibration()) {
        ESP_LOGE(TAG, "Fallo en perform_ref_calibration");
        LOG_MESSAGE_E(TAG,"Fallo en perform_ref_calibration");
        return false;
    }
    return true;
}

bool vl53l0x_init()
{
    if (!init_addresses()) {
        ESP_LOGE(TAG, "Fallo en init_addresses");
        LOG_MESSAGE_E(TAG,"Fallo en init_addresses");
        return false;
    }
    if (!init_config(VL53L0X_IDX_FIRST)) {
        ESP_LOGE(TAG, "Fallo en init_config");
        LOG_MESSAGE_E(TAG,"Fallo en init_config");
        return false;
    }
#ifdef VL53L0X_SECOND
    if (!init_config(VL53L0X_IDX_SECOND)) {
        return false;
    }
#endif
#ifdef VL53L0X_THIRD
    if (!init_config(VL53L0X_IDX_THIRD)) {
        return false;
    }
#endif
    return true;
}

esp_err_t vl53l0x_read_range_single(vl53l0x_idx_t idx, uint16_t *range)
{
    //i2c_set_slave_address(vl53l0x_infos[idx].addr);
    bool success = i2c_write_addr8_data8(0x80, 0x01);
    success &= i2c_write_addr8_data8(0xFF, 0x01);
    success &= i2c_write_addr8_data8(0x00, 0x00);
    success &= i2c_write_addr8_data8(0x91, stop_variable);
    success &= i2c_write_addr8_data8(0x00, 0x01);
    success &= i2c_write_addr8_data8(0xFF, 0x00);
    success &= i2c_write_addr8_data8(0x80, 0x00);
    
    if (!success) {
        return ESP_FAIL;
    }

    if (!i2c_write_addr8_data8(REG_SYSRANGE_START, 0x01)) {
        return ESP_FAIL;
    }

    uint8_t sysrange_start = 0;
    do {
        success = i2c_read_addr8_data8(REG_SYSRANGE_START, &sysrange_start);
    } while (success && (sysrange_start & 0x01));
    
    if (!success) {
        return ESP_FAIL;
    }

    uint8_t interrupt_status = 0;
    do {
        success = i2c_read_addr8_data8(REG_RESULT_INTERRUPT_STATUS, &interrupt_status);
    } while (success && ((interrupt_status & 0x07) == 0));
    
    if (!success) {
        return ESP_FAIL;
    }

    if (!i2c_read_addr8_data16(REG_RESULT_RANGE_STATUS + 10, range)) {
        return ESP_FAIL;
    }

    if (!i2c_write_addr8_data8(REG_SYSTEM_INTERRUPT_CLEAR, 0x01)) {
        return ESP_FAIL;
    }
    
    /* 8190 or 8191 may be returned when obstacle is out of range. */
    if (*range == 8190 || *range == 8191) {
        *range = VL53L0X_OUT_OF_RANGE;
    }

    return ESP_OK;
}

esp_err_t vl53l0x_reset() {

    esp_err_t err;

    err = gpio_set_level(GPIO_XSHUT_FIRST, 0); // Apagar
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Error setting GPIO_XSHUT_FIRST to 0: %s", esp_err_to_name(err));
        LOG_MESSAGE_E(TAG,"Error setting GPIO_XSHUT_FIRST to 0");
        return ESP_FAIL;
    }

    vTaskDelay(pdMS_TO_TICKS(30)); // Esperar 10ms
    
    err = gpio_set_level(GPIO_XSHUT_FIRST, 1); // Encender
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Error setting GPIO_XSHUT_FIRST to 1: %s", esp_err_to_name(err));
        LOG_MESSAGE_E(TAG,"Error setting GPIO_XSHUT_FIRST to 1");
        return ESP_FAIL;
    }

    vTaskDelay(pdMS_TO_TICKS(30)); // Esperar a que el sensor esté listo

    if (device_is_booted()) {
        if( vl53l0x_init() ){
            return ESP_OK;
        }
        ESP_LOGE(TAG, "Failed to restart and boot the VL53L0X");
        LOG_MESSAGE_E(TAG,"Failed to restart and boot the VL53L0X");
        return ESP_FAIL;
    } 
    else {
        ESP_LOGE(TAG, "VL53L0X no se inició correctamente después del reinicio");
        LOG_MESSAGE_E(TAG,"VL53L0X no se inició correctamente después del reinicio");
        return ESP_FAIL;
    }
}

// ============================================================================
// ENHANCED VL53L0X FUNCTIONS FOR MAPPING APPLICATIONS
// ============================================================================

// Additional registers for enhanced functionality
#define REG_MSRC_CONFIG_TIMEOUT_MACROP (0x46)
#define REG_PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI (0x51)
#define REG_PRE_RANGE_CONFIG_TIMEOUT_MACROP_LO (0x52)
#define REG_FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI (0x71)
#define REG_FINAL_RANGE_CONFIG_TIMEOUT_MACROP_LO (0x72)
#define REG_FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT (0x44)
#define REG_PRE_RANGE_CONFIG_SIGMA_THRESH_HI (0x61)
#define REG_PRE_RANGE_CONFIG_SIGMA_THRESH_LO (0x62)
#define REG_FINAL_RANGE_CONFIG_SIGMA_THRESH_HI (0x67)
#define REG_FINAL_RANGE_CONFIG_SIGMA_THRESH_LO (0x68)
#define REG_CROSSTALK_COMPENSATION_PEAK_RATE_MCPS (0x20)
#define REG_MSRC_CONFIG_CONTROL (0x60)
#define REG_PRE_RANGE_CONFIG_MIN_SNR (0x27)
#define REG_SYSTEM_INTERMEASUREMENT_PERIOD (0x04)

// Global variables for enhanced functionality
static struct {
    vl53l0x_mode_t mode;
    uint32_t timing_budget_us;
    uint32_t inter_measurement_period_ms;
    vl53l0x_data_ready_cb_t callback;
    gpio_num_t interrupt_gpio;
    bool interrupt_enabled;
    uint32_t measurement_count;
    uint32_t error_count;
} vl53l0x_state[3] = {0}; // Support up to 3 sensors

// Internal helper functions
static bool get_sequence_step_enables(uint8_t *enables);
static bool get_sequence_step_timeouts(uint8_t *timeouts);
static uint32_t get_measurement_timing_budget(vl53l0x_idx_t idx);
static bool set_measurement_timing_budget(vl53l0x_idx_t idx, uint32_t budget_us);
static uint16_t encode_timeout(uint32_t timeout_mclks);
static uint32_t decode_timeout(uint16_t encoded_timeout);
static uint32_t timeout_mclks_to_us(uint16_t timeout_mclks, uint8_t vcsel_period);
static uint16_t timeout_us_to_mclks(uint32_t timeout_us, uint8_t vcsel_period);
static uint8_t get_vcsel_pulse_period(uint8_t vcsel_period_type);
static bool set_vcsel_pulse_period(uint8_t vcsel_period_type, uint8_t period_pclks);
static void IRAM_ATTR vl53l0x_isr_handler(void* arg);

/**
 * @brief Configures the timing budget for measurements
 */
esp_err_t vl53l0x_set_timing_budget(vl53l0x_idx_t idx, vl53l0x_timing_budget_t budget_us)
{
    if (idx >= (sizeof(vl53l0x_state) / sizeof(vl53l0x_state[0]))) {
        return ESP_ERR_INVALID_ARG;
    }

    if (budget_us < 15000 || budget_us > 100000) {
        ESP_LOGE(TAG, "Invalid timing budget: %u us", budget_us);
        return ESP_ERR_INVALID_ARG;
    }

    if (!set_measurement_timing_budget(idx, budget_us)) {
        ESP_LOGE(TAG, "Failed to set timing budget");
        return ESP_FAIL;
    }

    vl53l0x_state[idx].timing_budget_us = budget_us;
    ESP_LOGI(TAG, "Timing budget set to %u us", budget_us);
    return ESP_OK;
}

/**
 * @brief Starts continuous ranging mode
 */
esp_err_t vl53l0x_start_continuous(vl53l0x_idx_t idx, uint32_t inter_measurement_period_ms)
{
    if (idx >= (sizeof(vl53l0x_state) / sizeof(vl53l0x_state[0]))) {
        return ESP_ERR_INVALID_ARG;
    }

    // Ensure inter-measurement period is at least as long as the timing budget
    uint32_t min_period_ms = (vl53l0x_state[idx].timing_budget_us + 5000) / 1000; // Add 5ms margin
    if (inter_measurement_period_ms < min_period_ms) {
        inter_measurement_period_ms = min_period_ms;
        ESP_LOGW(TAG, "Inter-measurement period adjusted to %lu ms", inter_measurement_period_ms);
    }

    // Set inter-measurement period
    if (!i2c_write_addr8_data16(REG_SYSTEM_INTERMEASUREMENT_PERIOD, inter_measurement_period_ms)) {
        ESP_LOGE(TAG, "Failed to set inter-measurement period");
        return ESP_FAIL;
    }

    // Start continuous mode
    if (!i2c_write_addr8_data8(REG_SYSRANGE_START, 0x02)) {
        ESP_LOGE(TAG, "Failed to start continuous mode");
        return ESP_FAIL;
    }

    vl53l0x_state[idx].mode = VL53L0X_MODE_CONTINUOUS;
    vl53l0x_state[idx].inter_measurement_period_ms = inter_measurement_period_ms;
    vl53l0x_state[idx].measurement_count = 0;
    vl53l0x_state[idx].error_count = 0;

    ESP_LOGI(TAG, "Continuous mode started with %lu ms period", inter_measurement_period_ms);
    return ESP_OK;
}

/**
 * @brief Stops continuous ranging mode
 */
esp_err_t vl53l0x_stop_continuous(vl53l0x_idx_t idx)
{
    if (idx >= (sizeof(vl53l0x_state) / sizeof(vl53l0x_state[0]))) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!i2c_write_addr8_data8(REG_SYSRANGE_START, 0x01)) {
        ESP_LOGE(TAG, "Failed to stop continuous mode");
        return ESP_FAIL;
    }

    vl53l0x_state[idx].mode = VL53L0X_MODE_SINGLE_SHOT;
    ESP_LOGI(TAG, "Continuous mode stopped");
    return ESP_OK;
}

/**
 * @brief Reads measurement data in continuous mode
 */
esp_err_t vl53l0x_read_range_continuous(vl53l0x_idx_t idx, vl53l0x_measurement_t *measurement)
{
    if (idx >= (sizeof(vl53l0x_state) / sizeof(vl53l0x_state[0])) || !measurement) {
        return ESP_ERR_INVALID_ARG;
    }

    // Initialize measurement structure
    memset(measurement, 0, sizeof(vl53l0x_measurement_t));
    measurement->timestamp_us = esp_timer_get_time();

    // Check if measurement is ready
    if (!vl53l0x_is_data_ready(idx)) {
        measurement->valid = false;
        return ESP_ERR_NOT_FOUND;
    }

    // Read range status
    uint8_t range_status = 0;
    if (!i2c_read_addr8_data8(REG_RESULT_RANGE_STATUS, &range_status)) {
        ESP_LOGE(TAG, "Failed to read range status");
        measurement->valid = false;
        vl53l0x_state[idx].error_count++;
        return ESP_FAIL;
    }

    measurement->range_status = range_status & 0x78;

    // Read range data
    uint16_t range_mm = 0;
    if (!i2c_read_addr8_data16(REG_RESULT_RANGE_STATUS + 10, &range_mm)) {
        ESP_LOGE(TAG, "Failed to read range data");
        measurement->valid = false;
        vl53l0x_state[idx].error_count++;
        return ESP_FAIL;
    }

    // Read signal rate
    uint16_t signal_rate = 0;
    if (!i2c_read_addr8_data16(REG_RESULT_RANGE_STATUS + 6, &signal_rate)) {
        ESP_LOGW(TAG, "Failed to read signal rate");
        signal_rate = 0;
    }

    // Read ambient rate
    uint16_t ambient_rate = 0;
    if (!i2c_read_addr8_data16(REG_RESULT_RANGE_STATUS + 8, &ambient_rate)) {
        ESP_LOGW(TAG, "Failed to read ambient rate");
        ambient_rate = 0;
    }

    // Clear interrupt
    if (!i2c_write_addr8_data8(REG_SYSTEM_INTERRUPT_CLEAR, 0x01)) {
        ESP_LOGW(TAG, "Failed to clear interrupt");
    }

    // Process range data
    if (range_mm == 8190 || range_mm == 8191) {
        measurement->range_mm = VL53L0X_OUT_OF_RANGE;
    } else {
        measurement->range_mm = range_mm;
    }

    measurement->signal_rate = signal_rate;
    measurement->ambient_rate = ambient_rate;
    measurement->valid = (measurement->range_status == 0) && (measurement->range_mm != VL53L0X_OUT_OF_RANGE);

    vl53l0x_state[idx].measurement_count++;
    
    return ESP_OK;
}

/**
 * @brief Checks if new measurement data is available
 */
bool vl53l0x_is_data_ready(vl53l0x_idx_t idx)
{
    if (idx >= (sizeof(vl53l0x_state) / sizeof(vl53l0x_state[0]))) {
        return false;
    }

    uint8_t interrupt_status = 0;
    if (!i2c_read_addr8_data8(REG_RESULT_INTERRUPT_STATUS, &interrupt_status)) {
        return false;
    }

    return (interrupt_status & 0x07) != 0;
}

/**
 * @brief Interrupt handler for VL53L0X data ready
 */
static void IRAM_ATTR vl53l0x_isr_handler(void* arg)
{
    vl53l0x_idx_t idx = (vl53l0x_idx_t)(uintptr_t)arg;
    
    if (vl53l0x_state[idx].callback) {
        vl53l0x_measurement_t measurement;
        
        // Quick non-blocking read - full processing in callback
        measurement.timestamp_us = esp_timer_get_time();
        measurement.valid = true;
        
        // Call user callback from ISR
        vl53l0x_state[idx].callback(idx, &measurement);
    }
}

/**
 * @brief Installs interrupt handler for data ready events
 */
esp_err_t vl53l0x_install_interrupt_handler(vl53l0x_idx_t idx, gpio_num_t gpio_num, vl53l0x_data_ready_cb_t callback)
{
    if (idx >= (sizeof(vl53l0x_state) / sizeof(vl53l0x_state[0])) || !callback) {
        return ESP_ERR_INVALID_ARG;
    }

    // Configure GPIO as input with pull-up
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << gpio_num),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_NEGEDGE  // VL53L0X interrupt is active low
    };

    esp_err_t err = gpio_config(&io_conf);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure interrupt GPIO");
        return err;
    }

    // Install ISR handler
    err = gpio_install_isr_service(ESP_INTR_FLAG_IRAM);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "Failed to install ISR service");
        return err;
    }

    err = gpio_isr_handler_add(gpio_num, vl53l0x_isr_handler, (void*)(uintptr_t)idx);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add ISR handler");
        return err;
    }

    // Store configuration
    vl53l0x_state[idx].callback = callback;
    vl53l0x_state[idx].interrupt_gpio = gpio_num;
    vl53l0x_state[idx].interrupt_enabled = true;

    ESP_LOGI(TAG, "Interrupt handler installed on GPIO %d", gpio_num);
    return ESP_OK;
}

/**
 * @brief Removes interrupt handler
 */
esp_err_t vl53l0x_remove_interrupt_handler(vl53l0x_idx_t idx)
{
    if (idx >= (sizeof(vl53l0x_state) / sizeof(vl53l0x_state[0]))) {
        return ESP_ERR_INVALID_ARG;
    }

    if (vl53l0x_state[idx].interrupt_enabled) {
        gpio_isr_handler_remove(vl53l0x_state[idx].interrupt_gpio);
        vl53l0x_state[idx].interrupt_enabled = false;
        vl53l0x_state[idx].callback = NULL;
        ESP_LOGI(TAG, "Interrupt handler removed");
    }

    return ESP_OK;
}

/**
 * @brief Configures signal and ambient rate limits
 */
esp_err_t vl53l0x_set_signal_rate_limit(vl53l0x_idx_t idx, float signal_rate_limit, float ambient_rate_limit)
{
    if (idx >= (sizeof(vl53l0x_state) / sizeof(vl53l0x_state[0]))) {
        return ESP_ERR_INVALID_ARG;
    }

    // Convert signal rate limit to fixed point (9.7 format)
    uint16_t signal_rate_fixed = (uint16_t)(signal_rate_limit * 128.0f);
    
    if (!i2c_write_addr8_data16(REG_FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT, signal_rate_fixed)) {
        ESP_LOGE(TAG, "Failed to set signal rate limit");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Signal rate limit set to %.2f MCPS", signal_rate_limit);
    return ESP_OK;
}

/**
 * @brief Gets current measurement rate capability
 */
float vl53l0x_get_max_measurement_rate(vl53l0x_idx_t idx)
{
    if (idx >= (sizeof(vl53l0x_state) / sizeof(vl53l0x_state[0]))) {
        return 0.0f;
    }

    uint32_t timing_budget = vl53l0x_state[idx].timing_budget_us;
    if (timing_budget == 0) {
        timing_budget = 30000; // Default 30ms
    }

    // Maximum rate is limited by timing budget + overhead
    float max_rate = 1000000.0f / (timing_budget + 5000); // 5ms overhead
    return max_rate;
}

// Helper functions implementation

static bool get_sequence_step_enables(uint8_t *enables)
{
    return i2c_read_addr8_data8(REG_SYSTEM_SEQUENCE_CONFIG, enables);
}

static bool get_sequence_step_timeouts(uint8_t *timeouts)
{
    // This is a simplified implementation
    // In a full implementation, you would read multiple registers
    *timeouts = 0;
    return true;
}

static bool set_measurement_timing_budget(vl53l0x_idx_t idx, uint32_t budget_us)
{
    uint16_t const StartOverhead = 1320; // microseconds
    uint16_t const EndOverhead = 960;
    uint16_t const MsrcOverhead = 660;
    uint16_t const TccOverhead = 590;
    uint16_t const DssOverhead = 690;
    uint16_t const PreRangeOverhead = 660;
    uint16_t const FinalRangeOverhead = 550;

    uint32_t const MinTimingBudget = 20000; // microseconds

    if (budget_us < MinTimingBudget) {
        return false;
    }

    uint32_t used_budget_us = StartOverhead + EndOverhead;

    uint8_t enables = 0;
    if (!get_sequence_step_enables(&enables)) {
        return false;
    }

    if (enables & RANGE_SEQUENCE_STEP_TCC) {
        used_budget_us += TccOverhead;
    }

    if (enables & RANGE_SEQUENCE_STEP_DSS) {
        used_budget_us += DssOverhead;
    }

    if (enables & RANGE_SEQUENCE_STEP_MSRC) {
        used_budget_us += MsrcOverhead;
    }

    if (enables & RANGE_SEQUENCE_STEP_PRE_RANGE) {
        used_budget_us += PreRangeOverhead;
    }

    if (enables & RANGE_SEQUENCE_STEP_FINAL_RANGE) {
        used_budget_us += FinalRangeOverhead;

        if (used_budget_us > budget_us) {
            return false;
        }

        uint32_t final_range_budget_us = budget_us - used_budget_us;

        uint8_t vcsel_period = get_vcsel_pulse_period(1); // Final range VCSEL period
        uint16_t final_range_timeout_mclks = timeout_us_to_mclks(final_range_budget_us, vcsel_period);

        if (final_range_timeout_mclks > 65535) {
            return false;
        }

        // Set final range timeout
        if (!i2c_write_addr8_data16(REG_FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI, 
                                   encode_timeout(final_range_timeout_mclks))) {
            return false;
        }
    }

    return true;
}

static uint16_t encode_timeout(uint32_t timeout_mclks)
{
    uint32_t ls_byte = 0;
    uint16_t ms_byte = 0;

    if (timeout_mclks > 0) {
        ls_byte = timeout_mclks - 1;
        while ((ls_byte & 0xFFFFFF00) > 0) {
            ls_byte >>= 1;
            ms_byte++;
        }
        return (ms_byte << 8) | (ls_byte & 0xFF);
    }
    return 0;
}

static uint32_t decode_timeout(uint16_t encoded_timeout)
{
    return (uint32_t)((encoded_timeout & 0xFF) << (encoded_timeout >> 8)) + 1;
}

static uint32_t timeout_mclks_to_us(uint16_t timeout_mclks, uint8_t vcsel_period)
{
    uint32_t macro_period_ns = (((uint32_t)2304 * (vcsel_period) * 1655) + 500) / 1000;
    return ((timeout_mclks * macro_period_ns) + 500) / 1000;
}

static uint16_t timeout_us_to_mclks(uint32_t timeout_us, uint8_t vcsel_period)
{
    uint32_t macro_period_ns = (((uint32_t)2304 * (vcsel_period) * 1655) + 500) / 1000;
    return (uint16_t)(((timeout_us * 1000) + (macro_period_ns / 2)) / macro_period_ns);
}

static uint8_t get_vcsel_pulse_period(uint8_t vcsel_period_type)
{
    uint8_t vcsel_period_reg = 0;
    
    if (vcsel_period_type == 0) {
        // Pre-range
        if (!i2c_read_addr8_data8(REG_PRE_RANGE_CONFIG_SIGMA_THRESH_HI, &vcsel_period_reg)) {
            return 12; // Default value
        }
    } else {
        // Final range
        if (!i2c_read_addr8_data8(REG_FINAL_RANGE_CONFIG_SIGMA_THRESH_HI, &vcsel_period_reg)) {
            return 12; // Default value
        }
    }
    
    return (vcsel_period_reg + 1) << 1;
}

static bool set_vcsel_pulse_period(uint8_t vcsel_period_type, uint8_t period_pclks)
{
    uint8_t vcsel_period_reg = (period_pclks >> 1) - 1;
    
    if (vcsel_period_type == 0) {
        // Pre-range
        return i2c_write_addr8_data8(REG_PRE_RANGE_CONFIG_SIGMA_THRESH_HI, vcsel_period_reg);
    } else {
        // Final range
        return i2c_write_addr8_data8(REG_FINAL_RANGE_CONFIG_SIGMA_THRESH_HI, vcsel_period_reg);
    }
}
