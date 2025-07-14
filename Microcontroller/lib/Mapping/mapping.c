#include "mapping.h"
#include "Servo/servo_compatibility.h"
#include "LiDAR/vl53l0x.h"
#include "esp_log.h"
#include "debug_helper.h"

#define MIN_DISTANCE 100

static const char *TAG = "MAPPING";
static esp_err_t getValue(uint16_t *);

esp_err_t mapping_init()
{

    esp_err_t err = ESP_OK;
    DEBUGING_ESP_LOG(ESP_LOGI(TAG, "Initializing Mapping"));

    DEBUGING_ESP_LOG(ESP_LOGI(TAG, "Initializing GPIO..."));
    err = gpio_init();
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Error in gpio_init: %s", esp_err_to_name(err));
        LOG_MESSAGE_E(TAG, "Error in gpio_init");
        return err;
    }

    DEBUGING_ESP_LOG(ESP_LOGI(TAG, "Initializing I2C..."));
    err = i2c_init();
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Error initializing I2C");
        LOG_MESSAGE_E(TAG, "Error initializing I2C");
        return ESP_FAIL;
    }

    DEBUGING_ESP_LOG(ESP_LOGI(TAG, "Initializing LiDAR..."));
    if (!vl53l0x_init())
    {
        ESP_LOGE(TAG, "Error initializing LiDAR(VL53L0X)");
        LOG_MESSAGE_E(TAG, "Error initializing LiDAR(VL53L0X)");
        return ESP_FAIL;
    }

    DEBUGING_ESP_LOG(ESP_LOGI(TAG, "Initializing ServoMotor..."));
    err = servo_initialize();
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Error initializing Servo");
        LOG_MESSAGE_E(TAG, "Error initializing Servo");
        return ESP_FAIL;
    }
    err = servo_start();
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Error Starting Servo");
        LOG_MESSAGE_E(TAG, "Error Starting Servo");
        return ESP_FAIL;
    }

    return err;
}

esp_err_t getMappingValue(int16_t *angle, uint16_t *distance)
{

    if (angle == NULL || distance == NULL)
    {
        ESP_LOGE(TAG, "NULL pointer passed to getMappingValue.");
        LOG_MESSAGE_E(TAG, "NULL pointer passed to getMappingValue.");
        return ESP_ERR_INVALID_ARG;
    }

    *angle = readAngle();
    if (*angle == -1)
        return ESP_ERR_INVALID_RESPONSE;
    esp_err_t err = getValue(distance);
    // esp_err_t err = ESP_OK;
    //*distance = 500;
    if (err == ESP_FAIL)
    {
        ESP_LOGW(TAG, "ERROR MAPPING: %s", esp_err_to_name(err));
        LOG_MESSAGE_W(TAG, "ERROR MAPPING");

        // //LLAMAR RUTINA DE REINICIO LIDAR
        ESP_LOGW(TAG, "Reiniciando LiDAR...");
        LOG_MESSAGE_E(TAG, "Reiniciando LiDAR...");
        esp_err_t err2 = vl53l0x_reset();
        if (err2 != ESP_OK)
        {
            ESP_LOGE(TAG, "Error restarting the LiDAR: %s", esp_err_to_name(err2));
            LOG_MESSAGE_E(TAG,"Error restarting the LiDAR");
            return ESP_FAIL;
        }
        vTaskDelay(20 / portTICK_PERIOD_MS);

        return err;
    }
    return err;
}

// static esp_err_t getValue(uint16_t *distance)
// {
//     esp_err_t success;
//     uint16_t val = 0;
//     uint8_t N = 3;
//     uint16_t values[N];
//     uint8_t valid_readings = 0; // Contador para lecturas válidas

//     for (uint8_t i = 0; i < N; i++)
//     {
// #ifndef VL53L0X
//         success = vl53l0x_read_range_single(VL53L0X_IDX_FIRST, &val);
//         if ((success == ESP_OK) && (val < VL53L0X_OUT_OF_RANGE && val >= MIN_DISTANCE))
//         {
//             values[i] = val;  // Almacenar el valor si es válido
//             valid_readings++; // Incrementar el contador de lecturas válidas
//         }
//         else
//         {
//             // Si la lectura no es exitosa o el valor está fuera de rango
//             ESP_LOGE(TAG, "Error reading range or invalid value: %d", val);
//         }
// #else
//         ESP_LOGE(TAG, "ERROR VL53L0X NOT DEFINED");
//         return ESP_FAIL; // Si VL53L0X no está definido, retornar error
// #endif
//     }
//     // Ordenar las mediciones de menor a mayor
//     // Verificar si se obtuvieron suficientes lecturas válidas
//     if (valid_readings < N/2)
//     {
//         ESP_LOGE(TAG, "Not enough valid readings. Only %d valid readings.", valid_readings);
//         //return ESP_ERR_INVALID_ARG; // Retornar error si no se obtuvieron suficientes mediciones válidas
//         return ESP_OK;
//     }

//     // Ordenar las mediciones de menor a mayor
//     for (uint8_t i = 0; i < valid_readings - 1; i++)
//     {
//         for (uint8_t j = i + 1; j < valid_readings; j++)
//         {
//             if (values[i] > values[j])
//             {
//                 // Intercambiar los valores
//                 uint16_t temp = values[i];
//                 values[i] = values[j];
//                 values[j] = temp;
//             }
//         }
//     }

//     // La mediana será el valor en la posición central del arreglo ordenado
//     *distance = values[valid_readings / 2];
//     return ESP_OK; // Índice 5 es la mediana en un arreglo de 10 elementos
// }

static esp_err_t getValue(uint16_t *distance)
{
    esp_err_t success;
    uint16_t val = 0;

#ifndef VL53L0X
    success = vl53l0x_read_range_single(VL53L0X_IDX_FIRST, &val);
    if (success != ESP_OK)
    {
        // Si la lectura no es exitosa o el valor está fuera de rango
        ESP_LOGE(TAG, "Error reading: %s", esp_err_to_name(success));
        LOG_MESSAGE_W(TAG,"Error reading");
        return ESP_FAIL;
    }
    else
    {
        val -= 38; // Calibración del valor obtenido
        if (val < VL53L0X_OUT_OF_RANGE && val >= MIN_DISTANCE)
        {
            *distance = val;
        }
        else
        {
            ESP_LOGE(TAG, "Invalid value: %d", val);
            //LOG_MESSAGE_E(TAG,"Invalid value");
            return ESP_ERR_INVALID_RESPONSE;
        }
    }
#else
    ESP_LOGE(TAG, "ERROR VL53L0X NOT DEFINED");
    LOG_MESSAGE_E(TAG,"ERROR VL53L0X NOT DEFINED");
    return ESP_FAIL; // Si VL53L0X no está definido, retornar error
#endif

    return ESP_OK; // Índice 5 es la mediana en un arreglo de 10 elementos
}

esp_err_t mapping_pause()
{
    esp_err_t err = ESP_OK;

    err = servo_pause();
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "ERROR STOPING SERVO");
        LOG_MESSAGE_E(TAG,"ERROR STOPPING SERVO");
        return ESP_FAIL;
    }
    return ESP_OK;
}

esp_err_t mapping_stop()
{
    return servo_stop();
}

esp_err_t mapping_restart()
{
    esp_err_t err = ESP_OK;

    err = servo_restart();
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "ERROR RESTARTING SERVO");
        LOG_MESSAGE_E(TAG,"ERROR RESTARTING SERVO");
        return ESP_FAIL;
    }
    return ESP_OK;
}