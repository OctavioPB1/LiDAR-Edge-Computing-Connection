// #include "esp_log.h"
// #include "cyclops_core.h"
// #include "freertos/FreeRTOS.h"
// #include "freertos/task.h"
// #include "mapping.h"
// #include "battery.h"
// #include "motors.h"
// #include "mqtt_handler.h"

// #include "esp_timer.h"
// #include "heap_trace_helper.h"

// static const char *TAG = "MAIN";

// int app_main(void)
// {
//         esp_err_t err;

//         heap_trace_start(HEAP_TRACE_ALL);

//         ESP_LOGI(TAG, "Iniciando Sistemas...");
//         err = system_init();
//         if (err != ESP_OK)
//         {
//                 ESP_LOGE(TAG, "Error Iniciando Sistemas:  %s", esp_err_to_name(err));
//                 return 1;
//         }
//         ESP_LOGI(TAG, "Sistemas Iniciado!");
//         // GENERO PUNTO DE RETORNO

//         ESP_LOGI(TAG, "Iniciando Tareas...");
//         err = createTasks();
//         if (err != ESP_OK)
//         {
//                 ESP_LOGE(TAG, "Error Iniciando Tareas:  %s", esp_err_to_name(err));
//                 return 1;
//         }
//         ESP_LOGI(TAG, "Tareas Iniciadas!");
//         /*
//         uint64_t time_now = esp_timer_get_time();
//         while(1){
//                 saveInstruction("Forward");
//                 vTaskDelay(50 / portTICK_PERIOD_MS);

//                 if ( (esp_timer_get_time() - time_now) > 120000000 ){
//                         ESP_LOGI(TAG, "Pasó 1 minuto $$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$");
//                         heap_caps_print_heap_info(MALLOC_CAP_8BIT);
//                 }
//         }*/

//         return 0;
// }

#include "esp_log.h"
#include "esp_system.h"
#include "cyclops_core.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "mapping.h"
#include "battery.h"
#include "motors.h"
#include "mqtt_handler.h"

#include "esp_timer.h"
#include "heap_trace_helper.h"

#include "servo_compatibility.h"
#include "i2c.h"

static const char *TAG = "MAIN";

int app_main(void)
{
        esp_err_t err;

        heap_trace_start(HEAP_TRACE_ALL);

        ESP_LOGI(TAG, "======= SERVO TEST CORREGIDO (1440us) =======");
        ESP_LOGI(TAG, "ESP-IDF Version: %s", esp_get_idf_version());

        ESP_LOGI(TAG, "Iniciando I2C...");
        err = i2c_init();
        if (err != ESP_OK)
        {
                ESP_LOGE(TAG, "Error Iniciando I2C:  %s", esp_err_to_name(err));
                return 1;
        }
        ESP_LOGI(TAG, "I2C Iniciado OK!");
        vTaskDelay(1000 / portTICK_PERIOD_MS);

        ESP_LOGI(TAG, "Iniciando Servo (GPIO14)...");
        err = servo_simple_initialize();
        if (err != ESP_OK)
        {
                ESP_LOGE(TAG, "Error Iniciando Servo:  %s", esp_err_to_name(err));
                ESP_LOGE(TAG, "FALLO CRITICO: Servo no inicializado!");
                return 1;
        }
        ESP_LOGI(TAG, "Servo Inicializado OK!");
        vTaskDelay(2000 / portTICK_PERIOD_MS);

        ESP_LOGI(TAG, "Habilitando Servo...");
        err = servo_simple_start();
        if (err != ESP_OK)
        {
                ESP_LOGE(TAG, "Error Starting Servo:  %s", esp_err_to_name(err));
                ESP_LOGE(TAG, "FALLO CRITICO: Servo no habilitado!");
                return 1;
        }
        ESP_LOGI(TAG, "Servo Habilitado OK - Deberia estar moviendose!");
        //vTaskDelay(3000 / portTICK_PERIOD_MS);

        for (int i = 0; i < 4; i++)
        {
                ESP_LOGI(TAG, "Leyendo angulo actual: %d grados", readAngle_simple());
                err = servo_simple_pause();
                if (err != ESP_OK)
                {
                        ESP_LOGE(TAG, "Error Pausing Servo:  %s", esp_err_to_name(err));
                }
                vTaskDelay(2000 / portTICK_PERIOD_MS);
                err = servo_simple_start();
                if (err != ESP_OK)
                {
                        ESP_LOGE(TAG, "Error Starting Servo:  %s", esp_err_to_name(err));
                }
                vTaskDelay(2000 / portTICK_PERIOD_MS);
        }

        ESP_LOGI(TAG, "===== PRUEBA 1: Invertir direccion =====");
        servo_simple_invert();
        ESP_LOGI(TAG, "Direccion invertida - Servo deberia cambiar direccion");
        vTaskDelay(5000 / portTICK_PERIOD_MS);

        ESP_LOGI(TAG, "===== PRUEBA 2: Invertir nuevamente =====");
        servo_simple_invert();
        ESP_LOGI(TAG, "Direccion restaurada - Servo vuelve a direccion original");
        vTaskDelay(5000 / portTICK_PERIOD_MS);
        
        ESP_LOGI(TAG, "===== PRUEBA 3: Pausar servo =====");
        err = servo_simple_pause();
        if (err != ESP_OK)
        {
                ESP_LOGE(TAG, "Error Pausing Servo:  %s", esp_err_to_name(err));
        } else {
                ESP_LOGI(TAG, "Servo Pausado - Deberia haberse detenido");
        }
        vTaskDelay(3000 / portTICK_PERIOD_MS);

        ESP_LOGI(TAG, "===== PRUEBA 4: Detener servo completamente =====");
        err = servo_simple_stop();
        if (err != ESP_OK)
        {
                ESP_LOGE(TAG, "Error Stopping Servo:  %s", esp_err_to_name(err));
        } else {
                ESP_LOGI(TAG, "Servo Detenido Completamente");
        }
        vTaskDelay(2000 / portTICK_PERIOD_MS);

        ESP_LOGI(TAG, "===== TEST COMPLETADO =====");
        ESP_LOGI(TAG, "Si el servo no se movio:");
        ESP_LOGI(TAG, "1. Verificar conexion de alimentacion (5V)");
        ESP_LOGI(TAG, "2. Verificar conexion de señal en GPIO14");
        ESP_LOGI(TAG, "3. Verificar que el servo sea de rotacion continua");
        ESP_LOGI(TAG, "4. Revisar logs de errores arriba");

        return 0;
}