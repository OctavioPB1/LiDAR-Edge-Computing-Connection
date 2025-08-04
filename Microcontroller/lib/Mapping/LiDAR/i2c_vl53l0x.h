#ifndef _I2C_VL53L0X_H_
#define _I2C_VL53L0X_H_

#include "esp_err.h"
#include <stdbool.h>
#include "i2c.h"


/**
 * Wrapper functions for reading from registers with different address
 * and data sizes.
 * @return True if success, False if error
 * NOTE: Reads the most significant byte first if multi-byte data.
 * NOTE: Polling-based
 */
bool i2c_read_addr8_data8(uint8_t , uint8_t *);
bool i2c_read_addr8_data16(uint8_t , uint16_t *);
bool i2c_read_addr16_data8(uint16_t , uint8_t *);
bool i2c_read_addr16_data16(uint16_t , uint16_t *);
bool i2c_read_addr8_data32(uint8_t , uint32_t *);
bool i2c_read_addr16_data32(uint16_t , uint32_t *);

/**
 * Read byte_count bytes from address addr
 * NOTE: Polling-based
 */
bool i2c_read_addr8_bytes(uint8_t , uint8_t *, uint16_t );

/**
 * Wrapper functions for writing to registers with different address
 * and data sizes.
 * @return True if success, False if error
 * NOTE: Writes the most significant byte if multi-byte data.
 * NOTE: Polling-based
 */
bool i2c_write_addr8_data8(uint8_t , uint8_t);
bool i2c_write_addr8_data16(uint8_t , uint16_t);
bool i2c_write_addr16_data8(uint16_t , uint8_t);
bool i2c_write_addr16_data16(uint16_t , uint16_t);

/**
 * Write an array of bytes of size byte_count to address addr.
 * NOTE: Polling-based
 */
bool i2c_write_addr8_bytes(uint8_t, uint8_t *, uint16_t);

#endif