#include "i2c_vl53l0x.h"
#include "esp_log.h"
#include "debug_helper.h"


#define DEFAULT_SLAVE_ADDRESS (0x29)

static const char *TAG = "VL53L0X_I2C";


typedef enum
{
    ADDR_SIZE_8BIT,
    ADDR_SIZE_16BIT
} addr_size_t;

typedef enum
{
    REG_SIZE_8BIT,
    REG_SIZE_16BIT,
    REG_SIZE_32BIT
} reg_size_t;

/* Read a register of size reg_size at address addr.
 * NOTE: The bytes are read from MSB to LSB. */
static bool read_reg(uint8_t slave_addr, uint16_t addr, reg_size_t reg_size, uint8_t *data) {


    bool success = false;

    if(!i2c_get_bus()){
        ESP_LOGE(TAG,"Error getting I2C Bus");
        LOG_MESSAGE_E(TAG, "Error getting I2C Bus");
        return false;
    }
    
    DEBUGING_ESP_LOG(ESP_LOGI(TAG, "Crear un comando I2C"));
    
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    if (cmd == NULL) {
        ESP_LOGE(TAG, "Error al crear comando I2C");
        LOG_MESSAGE_E(TAG, "Error al crear comando I2C");
        i2c_give_bus();
        return false;
    }

    DEBUGING_ESP_LOG(ESP_LOGI(TAG, "Iniciar la transferencia I2C"));
    i2c_master_start(cmd);

    DEBUGING_ESP_LOG(ESP_LOGI(TAG, "Escribir la dirección del dispositivo en el bus (modo escritura)"));
    if (i2c_master_write_byte(cmd, (slave_addr << 1) | I2C_MASTER_WRITE, true) != ESP_OK) {
        ESP_LOGE(TAG, "Error al escribir la dirección del dispositivo");
        LOG_MESSAGE_E(TAG, "Error al escribir la dirección del dispositivo");
        i2c_cmd_link_delete(cmd);
        i2c_give_bus();
        return false;
    }

    DEBUGING_ESP_LOG(ESP_LOGI(TAG, "Escribir el registro que queremos leer"));
    if (i2c_master_write_byte(cmd, addr, true) != ESP_OK) {
        ESP_LOGE(TAG, "Error al escribir el registro");
        LOG_MESSAGE_E(TAG, "Error al escribir el registro");
        i2c_cmd_link_delete(cmd);
        i2c_give_bus();
        return false;
    }

    DEBUGING_ESP_LOG(ESP_LOGI(TAG, "Enviar el comando de repetición para leer (modo lectura)"));
    i2c_master_start(cmd);
    if (i2c_master_write_byte(cmd, (slave_addr << 1) | I2C_MASTER_READ, true) != ESP_OK) {
        ESP_LOGE(TAG, "Error al escribir la dirección del dispositivo en modo lectura");
        LOG_MESSAGE_E(TAG, "Error al escribir la dirección del dispositivo en modo lectura");
        i2c_cmd_link_delete(cmd);
        i2c_give_bus();
        return false;
    }

    DEBUGING_ESP_LOG(ESP_LOGI(TAG, "Leer los datos del registro"));
    switch (reg_size) {
        case REG_SIZE_8BIT:
            if (i2c_master_read_byte(cmd, data, I2C_MASTER_LAST_NACK) != ESP_OK) {
                ESP_LOGE(TAG, "Error en la lectura de 8 bits");
                LOG_MESSAGE_E(TAG, "Error en la lectura de 8 bits");
            } else {
                success = true;
            }
            break;
        case REG_SIZE_16BIT:
            if (i2c_master_read_byte(cmd, data + 1, I2C_MASTER_ACK) == ESP_OK &&
                i2c_master_read_byte(cmd, data, I2C_MASTER_LAST_NACK) == ESP_OK) {
                success = true;
            } else {
                ESP_LOGE(TAG, "Error en la lectura de 16 bits");
                LOG_MESSAGE_E(TAG, "Error en la lectura de 16 bits");
            }
            break;
        case REG_SIZE_32BIT:
            if (i2c_master_read_byte(cmd, data + 3, I2C_MASTER_ACK) == ESP_OK &&
                i2c_master_read_byte(cmd, data + 2, I2C_MASTER_ACK) == ESP_OK &&
                i2c_master_read_byte(cmd, data + 1, I2C_MASTER_ACK) == ESP_OK &&
                i2c_master_read_byte(cmd, data, I2C_MASTER_LAST_NACK) == ESP_OK) {
                success = true;
            } else {
                ESP_LOGE(TAG, "Error en la lectura de 32 bits");
                LOG_MESSAGE_E(TAG, "Error en la lectura de 32 bits");
            }
            break;
    }

    DEBUGING_ESP_LOG(ESP_LOGI(TAG, "Terminar la transferencia I2C"));
    if (i2c_master_stop(cmd) != ESP_OK){
        ESP_LOGE(TAG, "Error en el cierre de la transferencia con el Master");
        LOG_MESSAGE_E(TAG, "Error en el cierre de la transferencia con el Master");
        i2c_give_bus();
        return false;
    }

    DEBUGING_ESP_LOG(ESP_LOGI(TAG, "Enviar el comando I2C y verificar el éxito"));
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(1000));
    if (ret == ESP_OK) {
        success = true;
    } else {
        ESP_LOGE(TAG, "Error en la transferencia I2C en read_reg: %s", esp_err_to_name(ret));
        LOG_MESSAGE_E(TAG, "Error en la transferencia I2C en read_reg");
        success = false;
    } 

    DEBUGING_ESP_LOG(ESP_LOGI(TAG, "Liberar el comando I2C"));
    i2c_cmd_link_delete(cmd);

    if(!i2c_give_bus()){
        return false;
    }
    return success;
}

static bool read_reg_bytes(uint8_t slave_addr, addr_size_t addr_size, uint16_t addr, uint8_t *bytes, uint16_t byte_count) {
    bool success = false;

    if(!i2c_get_bus()){
        ESP_LOGE(TAG,"Error getting I2C Bus");
        LOG_MESSAGE_E(TAG, "Error getting I2C Bus");
        return false;
    }

    // Crear un comando I2C
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    if (cmd == NULL) {
        ESP_LOGE(TAG, "Error al crear comando I2C");
        LOG_MESSAGE_E(TAG, "Error al crear comando I2C");
        i2c_give_bus();
        return false;
    }

    // Iniciar la transferencia I2C en modo escritura para enviar la dirección del registro
    i2c_master_start(cmd);
    if (i2c_master_write_byte(cmd, (slave_addr << 1) | I2C_MASTER_WRITE, true) != ESP_OK) {
        ESP_LOGE(TAG, "Error al escribir la dirección del dispositivo en modo escritura");
        LOG_MESSAGE_E(TAG, "Error al escribir la dirección del dispositivo en modo escritura");
        i2c_cmd_link_delete(cmd);
        i2c_give_bus();
        return false;
    }

    // Enviar la dirección del registro (1 o 2 bytes según addr_size)
    if (addr_size == ADDR_SIZE_16BIT) {
        if (i2c_master_write_byte(cmd, (addr >> 8) & 0xFF, true) != ESP_OK) {
            ESP_LOGE(TAG, "Error al enviar el byte alto de la dirección");
        LOG_MESSAGE_E(TAG, "Error al enviar el byte alto de la dirección");
            i2c_cmd_link_delete(cmd);
            i2c_give_bus();
            return false;
        }
    }
    if (i2c_master_write_byte(cmd, addr & 0xFF, true) != ESP_OK) {
        ESP_LOGE(TAG, "Error al enviar el byte bajo de la dirección");
        LOG_MESSAGE_E(TAG, "Error al enviar el byte bajo de la dirección");
        i2c_cmd_link_delete(cmd);
        i2c_give_bus();
        return false;
    }

    // Repetir la condición de inicio para cambiar a modo lectura
    i2c_master_start(cmd);
    if (i2c_master_write_byte(cmd, (slave_addr << 1) | I2C_MASTER_READ, true) != ESP_OK) {
        ESP_LOGE(TAG, "Error al cambiar a modo lectura");
        LOG_MESSAGE_E(TAG, "Error al cambiar a modo lectura");
        i2c_cmd_link_delete(cmd);
        i2c_give_bus();
        return false;
    }

    // Leer los bytes en el buffer
    for (int i = 0; i < byte_count; i++) {
        if (i == byte_count - 1) {
            // Último byte debe terminar con NACK
            if (i2c_master_read_byte(cmd, &bytes[i], I2C_MASTER_NACK) != ESP_OK) {
                ESP_LOGE(TAG, "Error al leer el último byte");
                LOG_MESSAGE_E(TAG, "Error al leer el último byte");
                i2c_cmd_link_delete(cmd);
                i2c_give_bus();
                return false;
            }
        } else {
            // Leer el byte con ACK
            if (i2c_master_read_byte(cmd, &bytes[i], I2C_MASTER_ACK) != ESP_OK) {
                ESP_LOGE(TAG, "Error al leer un byte con ACK");
                LOG_MESSAGE_E(TAG, "Error al leer un byte con ACK");
                i2c_cmd_link_delete(cmd);
                i2c_give_bus();
                return false;
            }
        }
    }

    // Finalizar la transferencia I2C
    if (i2c_master_stop(cmd) != ESP_OK) {
        ESP_LOGE(TAG, "Error al finalizar la transferencia");
        LOG_MESSAGE_E(TAG, "Error al finalizar la transferencia");
        i2c_give_bus();
        return false;
    }

    // Ejecutar el comando y verificar si fue exitoso
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));
    if (ret == ESP_OK) {
        success = true;
    } else {
        ESP_LOGE(TAG, "Error en la transferencia I2C en read_reg_bytes: %s", esp_err_to_name(ret));
        LOG_MESSAGE_E(TAG, "Error en la transferencia I2C en read_reg_bytes");
    }

    // Liberar el comando I2C
    i2c_cmd_link_delete(cmd);

    if(!i2c_give_bus()){
        return false;
    }
    return success;
}

/* Write data to a register of size reg_size at address addr.
 * NOTE: Writes the most significant byte (MSB) first. */
static bool write_reg(uint8_t slave_addr, addr_size_t addr_size, uint16_t addr, reg_size_t reg_size, uint16_t data) {
    bool success = false;

    if(!i2c_get_bus()){
        ESP_LOGE(TAG,"Error getting I2C Bus");
        LOG_MESSAGE_E(TAG, "Error getting I2C Bus");
        return false;
    }

    // Crear un comando I2C
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    if (cmd == NULL) {
        ESP_LOGE(TAG, "Error al crear comando I2C");
        LOG_MESSAGE_E(TAG, "Error al crear comando I2C");
        i2c_give_bus();
        return false;
    }

    // Iniciar la transferencia I2C
    i2c_master_start(cmd);

    // Escribir la dirección del dispositivo en el bus (modo escritura)
    if (i2c_master_write_byte(cmd, (slave_addr << 1) | I2C_MASTER_WRITE, true) != ESP_OK) {
        ESP_LOGE(TAG, "Error al escribir la dirección del dispositivo en modo escritura");
        LOG_MESSAGE_E(TAG, "Error al escribir la dirección del dispositivo en modo escritura");
        i2c_cmd_link_delete(cmd);
        i2c_give_bus();
        return false;
    }

    // Escribir la dirección del registro según el tamaño de la dirección
    if (addr_size == ADDR_SIZE_16BIT) {
        if (i2c_master_write_byte(cmd, (addr >> 8) & 0xFF, true) != ESP_OK) {
            ESP_LOGE(TAG, "Error al escribir el byte alto de la dirección");
            LOG_MESSAGE_E(TAG, "Error al escribir el byte alto de la dirección");
            i2c_cmd_link_delete(cmd);
            i2c_give_bus();
            return false;
        }
    }
    if (i2c_master_write_byte(cmd, addr & 0xFF, true) != ESP_OK) {
        ESP_LOGE(TAG, "Error al escribir el byte bajo de la dirección");
        LOG_MESSAGE_E(TAG, "Error al escribir el byte bajo de la dirección");
        i2c_cmd_link_delete(cmd);
        i2c_give_bus();
        return false;
    }

    // Escribir los datos según el tamaño del registro
    switch (reg_size) {
        case REG_SIZE_8BIT:
            if (i2c_master_write_byte(cmd, data & 0xFF, true) != ESP_OK) {
                ESP_LOGE(TAG, "Error al escribir el dato de 8 bits");
                LOG_MESSAGE_E(TAG,"Error al escribir el dato de 8 bits");
                i2c_cmd_link_delete(cmd);
                i2c_give_bus();
                return false;
            }
            success = true;
            break;
        case REG_SIZE_16BIT:
            if (i2c_master_write_byte(cmd, (data >> 8) & 0xFF, true) != ESP_OK ||
                i2c_master_write_byte(cmd, data & 0xFF, true) != ESP_OK) {
                ESP_LOGE(TAG, "Error al escribir el dato de 16 bits");
                LOG_MESSAGE_E(TAG,"Error al escribir el dato de 16 bits");
                i2c_cmd_link_delete(cmd);
                i2c_give_bus();
                return false;
            }
            success = true;
            break;
        case REG_SIZE_32BIT:
            // Enviar los cuatro bytes de datos, del más significativo al menos significativo
            if (i2c_master_write_byte(cmd, (data >> 24) & 0xFF, true) != ESP_OK ||
                i2c_master_write_byte(cmd, (data >> 16) & 0xFF, true) != ESP_OK ||
                i2c_master_write_byte(cmd, (data >> 8) & 0xFF, true) != ESP_OK ||
                i2c_master_write_byte(cmd, data & 0xFF, true) != ESP_OK) {
                ESP_LOGE(TAG, "Error al escribir el dato de 32 bits");
                LOG_MESSAGE_E(TAG,"Error al escribir el dato de 32 bits");
                i2c_cmd_link_delete(cmd);
                i2c_give_bus();
                return false;
            }
            success = true;
            break;
    }

    // Terminar la transferencia I2C
    if (i2c_master_stop(cmd) != ESP_OK) {
        ESP_LOGE(TAG, "Error en el cierre de la transferencia con el Master");
        LOG_MESSAGE_E(TAG,"Error en el cierre de la transferencia con el Master");
        success = false;
    }

    // Enviar el comando I2C y verificar el éxito
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));
    if (ret == ESP_OK) {
        success = true;
    } else {
        ESP_LOGE(TAG, "Error en la transferencia I2C en write_reg: %s", esp_err_to_name(ret));
        LOG_MESSAGE_E(TAG,"Error en la transferencia I2C en write_reg");
        success = false;
    }

    // Liberar el comando I2C
    i2c_cmd_link_delete(cmd);

    if(!i2c_give_bus()){
        return false;
    }

    return success;
}

static bool write_reg_bytes(uint8_t slave_addr, uint16_t addr, uint8_t *bytes, uint16_t byte_count) {
    bool success = false;

    if(!i2c_get_bus()){
        ESP_LOGE(TAG,"Error getting I2C Bus");
        LOG_MESSAGE_E(TAG,"Error getting I2C Bus");
        return false;
    }

    // Crear un comando I2C para la transferencia
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    if (cmd == NULL) {
        ESP_LOGE(TAG, "Error al crear comando I2C");
        LOG_MESSAGE_E(TAG,"Error al crear comando I2C");
        i2c_give_bus();
        return false;
    }

    // Iniciar la transferencia I2C
    i2c_master_start(cmd);

    // Escribir la dirección del dispositivo en el bus (modo escritura)
    if (i2c_master_write_byte(cmd, (slave_addr << 1) | I2C_MASTER_WRITE, true) != ESP_OK) {
        ESP_LOGE(TAG, "Error al escribir la dirección del dispositivo en modo escritura");
        LOG_MESSAGE_E(TAG,"Error al escribir la dirección del dispositivo en modo escritura");
        i2c_cmd_link_delete(cmd);
        i2c_give_bus();
        return false;
    }

    // Escribir la dirección del registro
    if (i2c_master_write_byte(cmd, (addr >> 8) & 0xFF, true) != ESP_OK || 
        i2c_master_write_byte(cmd, addr & 0xFF, true) != ESP_OK) {
        ESP_LOGE(TAG, "Error al escribir la dirección del registro");
        LOG_MESSAGE_E(TAG,"Error al escribir la dirección del registro");
        i2c_cmd_link_delete(cmd);
        i2c_give_bus();
        return false;
    }

    // Escribir los bytes al registro del dispositivo
    for (uint16_t i = 0; i < byte_count; i++) {
        if (i2c_master_write_byte(cmd, bytes[i], true) != ESP_OK) {
            ESP_LOGE(TAG, "Error al escribir el byte %d en el dispositivo", i);
            LOG_MESSAGE_E(TAG,"Error al escribir un byte en el dispositivo");
            i2c_cmd_link_delete(cmd);
            i2c_give_bus();
            return false;
        }
    }

    // Terminar la transferencia I2C
    if (i2c_master_stop(cmd) != ESP_OK) {
        ESP_LOGE(TAG, "Error en el cierre de la transferencia con el Master");
        LOG_MESSAGE_E(TAG,"Error en el cierre de la transferencia con el Master");
        success = false;
    } else {
        success = true;
    }

    // Enviar el comando I2C y verificar el éxito
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error en la transferencia I2C en write_reg_bytes: %s", esp_err_to_name(ret));
        LOG_MESSAGE_E(TAG,"Error en la transferencia I2C en write_reg_bytes");
        success = false;
    }

    // Liberar el comando I2C
    i2c_cmd_link_delete(cmd);

    if(!i2c_give_bus()){
        return false;
    }

    return success;
}


bool i2c_read_addr8_data8(uint8_t addr, uint8_t *data)
{
    return read_reg(0x29, addr, REG_SIZE_8BIT, data);
}

bool i2c_read_addr8_data16(uint8_t addr, uint16_t *data)
{
    return read_reg(0x29, addr, REG_SIZE_16BIT, (uint8_t *)data);
}

bool i2c_read_addr16_data8(uint16_t addr, uint8_t *data)
{
    return read_reg(0x29, addr, REG_SIZE_8BIT, data);
}

bool i2c_read_addr16_data16(uint16_t addr, uint16_t *data)
{
    return read_reg(0x29, addr, REG_SIZE_16BIT, (uint8_t *)data);
}

bool i2c_read_addr8_data32(uint16_t addr, uint32_t *data)
{   
    DEBUGING_ESP_LOG(ESP_LOGE(TAG, "ALERTA: El registro de dirección 8bits 32bits no es soportado"));
    LOG_MESSAGE_E(TAG,"ALERTA: El registro de dirección 8bits 32bits no es soportado");
    return read_reg(0x29, addr, REG_SIZE_32BIT, (uint8_t *)data);
}

bool i2c_read_addr16_data32(uint16_t addr, uint32_t *data)
{
    DEBUGING_ESP_LOG(ESP_LOGE(TAG, "ALERTA: El registro de dirección 16bits 32bits no es soportado"));
    LOG_MESSAGE_E(TAG,"ALERTA: El registro de dirección 16bits 32bits no es soportado");
    return read_reg(0x29, addr, REG_SIZE_32BIT, (uint8_t *)data);
}

bool i2c_read_addr8_bytes(uint8_t start_addr, uint8_t *bytes, uint16_t byte_count)
{
    return read_reg_bytes(0x29, ADDR_SIZE_8BIT, start_addr, bytes, byte_count);
}

bool i2c_write_addr8_data8(uint8_t addr, uint8_t value)
{
    return write_reg(0x29, ADDR_SIZE_8BIT, addr, REG_SIZE_8BIT, value);
}

bool i2c_write_addr8_data16(uint8_t addr, uint16_t value)
{
    return write_reg(0x29, ADDR_SIZE_8BIT, addr, REG_SIZE_16BIT, value);
}

bool i2c_write_addr16_data8(uint16_t addr, uint8_t value)
{
    return write_reg(0x29, ADDR_SIZE_16BIT, addr, REG_SIZE_8BIT, value);
}

bool i2c_write_addr16_data16(uint16_t addr, uint16_t value)
{
    return write_reg(0x29, ADDR_SIZE_16BIT, addr, REG_SIZE_16BIT, value);
}

bool i2c_write_addr8_bytes(uint8_t start_addr, uint8_t *bytes, uint16_t byte_count)
{
    return write_reg_bytes(0x29, start_addr, bytes, byte_count);
}
