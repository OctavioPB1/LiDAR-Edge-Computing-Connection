#include "esp_log.h"
#include "cyclops_core.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "mapping.h"
#include "battery.h"
#include "motors.h"
#include "mqtt_handler.h"

#include "esp_timer.h"
#include "heap_trace_helper.h"

static const char *TAG = "MAIN";

int app_main(void)
{
        esp_err_t err;

        heap_trace_start(HEAP_TRACE_ALL);

        ESP_LOGI(TAG, "Iniciando Sistemas...");
        err = system_init();
        if (err != ESP_OK)
        {
                ESP_LOGE(TAG, "Error Iniciando Sistemas:  %s", esp_err_to_name(err));
                return 1;
        }
        ESP_LOGI(TAG, "Sistemas Iniciado!");
        // GENERO PUNTO DE RETORNO

        ESP_LOGI(TAG, "Iniciando Tareas...");
        err = createTasks();
        if (err != ESP_OK)
        {
                ESP_LOGE(TAG, "Error Iniciando Tareas:  %s", esp_err_to_name(err));
                return 1;
        }
        ESP_LOGI(TAG, "Tareas Iniciadas!");
        /*
        uint64_t time_now = esp_timer_get_time();
        while(1){
                saveInstruction("Forward");
                vTaskDelay(50 / portTICK_PERIOD_MS);

                if ( (esp_timer_get_time() - time_now) > 120000000 ){
                        ESP_LOGI(TAG, "Pasó 1 minuto $$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$");
                        heap_caps_print_heap_info(MALLOC_CAP_8BIT);
                }
        }*/

        return 0;
}

// #include "esp_log.h"
// #include "esp_system.h"
// #include "cyclops_core.h"
// #include "freertos/FreeRTOS.h"
// #include "freertos/task.h"
// #include "mapping.h"
// #include "battery.h"
// #include "motors.h"
// #include "mqtt_handler.h"

// #include "esp_timer.h"
// #include "heap_trace_helper.h"

// #include "i2c.h"
// // Include the generic servo library for SG90
// #include "Servo/servo_compatibility.h"

// static const char *TAG = "SG90_TEST";

// // SG90 Servo Specifications
// // - Operating Voltage: 4.8V~6V
// // - Operating Speed: 0.1sec/60degree (4.8V), 0.08sec/60degree (6V)
// // - Stall Torque: 1.8kg.cm (4.8V), 2.5kg.cm (6V)
// // - Operating Angle: 0-180 degrees
// // - Required Pulse: 500us-2500us (typical)
// // - Frequency: 50Hz

// int app_main(void)
// {
//     esp_err_t ret;

//     ESP_LOGI(TAG, "======= PROGRAMA DE PRUEBA SERVO SG90 =======");
//     ESP_LOGI(TAG, "ESP-IDF Version: %s", esp_get_idf_version());
//     ESP_LOGI(TAG, "Servo: SG90 (Servo de posición 0-180°)");
//     ESP_LOGI(TAG, "GPIO: 14");
//     ESP_LOGI(TAG, "Alimentación: 5V");

//     // Inicializar el servo SG90
//     ESP_LOGI(TAG, "Inicializando servo SG90...");
//     ret = servo_simple_initialize();
//     if (ret != ESP_OK) {
//         ESP_LOGE(TAG, "Error inicializando SG90: %s", esp_err_to_name(ret));
//         return 1;
//     }
//     ESP_LOGI(TAG, "SG90 inicializado correctamente!");

//     // Habilitar el servo
//     ESP_LOGI(TAG, "Habilitando servo SG90...");
//     ret = servo_simple_start();
//     if (ret != ESP_OK) {
//         ESP_LOGE(TAG, "Error habilitando SG90: %s", esp_err_to_name(ret));
//         servo_simple_stop();
//         return 1;
//     }
//     ESP_LOGI(TAG, "SG90 habilitado - Listo para pruebas!");

//     // Esperar un momento para estabilización
//     vTaskDelay(1000 / portTICK_PERIOD_MS);

//     // ===== PRUEBA 1: Posiciones básicas =====
//     ESP_LOGI(TAG, "===== PRUEBA 1: Posiciones básicas =====");
    
//     // Posición 0 grados
//     ESP_LOGI(TAG, "Moviendo a 0 grados...");
//     ret = servo_simple_set_position(SERVO_POSITION_MIN);
//     if (ret != ESP_OK) {
//         ESP_LOGE(TAG, "Error moviendo a 0°: %s", esp_err_to_name(ret));
//     }
//     vTaskDelay(1000 / portTICK_PERIOD_MS);

//     // Posición 90 grados (centro)
//     ESP_LOGI(TAG, "Moviendo a 90 grados (centro)...");
//     ret = servo_simple_set_position(SERVO_POSITION_CENTER);
//     if (ret != ESP_OK) {
//         ESP_LOGE(TAG, "Error moviendo a 90°: %s", esp_err_to_name(ret));
//     }
//     vTaskDelay(1000 / portTICK_PERIOD_MS);

//     // Posición 180 grados
//     ESP_LOGI(TAG, "Moviendo a 180 grados...");
//     ret = servo_simple_set_position(SERVO_POSITION_MAX);
//     if (ret != ESP_OK) {
//         ESP_LOGE(TAG, "Error moviendo a 180°: %s", esp_err_to_name(ret));
//     }
//     vTaskDelay(1000 / portTICK_PERIOD_MS);

//     // ===== PRUEBA 2: Barrido completo =====
//     ESP_LOGI(TAG, "===== PRUEBA 2: Barrido completo (0-180-0) =====");
    
//     // Barrido de 0 a 180 grados
//     ESP_LOGI(TAG, "Barrido de 0 a 180 grados...");
//     for (int angle = 0; angle <= 180; angle += 2) {
//         ESP_LOGI(TAG, "Posición: %d°", angle);
//         servo_simple_set_position(angle);
//         vTaskDelay(300 / portTICK_PERIOD_MS);
//     }

//     // Barrido de 180 a 0 grados
//     ESP_LOGI(TAG, "Barrido de 180 a 0 grados...");
//     for (int angle = 180; angle >= 0; angle -= 2) {
//         ESP_LOGI(TAG, "Posición: %d°", angle);
//         servo_simple_set_position(angle);
//         vTaskDelay(300 / portTICK_PERIOD_MS);
//     }

//     // ===== PRUEBA 3: Movimiento suave =====
//     ESP_LOGI(TAG, "===== PRUEBA 3: Movimiento suave =====");
    
//     ESP_LOGI(TAG, "Movimiento suave de 0 a 180 grados (3 segundos)...");
//     servo_simple_continuous_sweep();
    
//     ESP_LOGI(TAG, "Movimiento suave de 180 a 0 grados (3 segundos)...");
//     servo_simple_continuous_sweep();

//     // ===== PRUEBA 4: Posiciones específicas =====
//     ESP_LOGI(TAG, "===== PRUEBA 4: Posiciones específicas =====");
    
//     uint16_t test_positions[] = {45, 135, 30, 150, 60, 120, 90};
//     int num_positions = sizeof(test_positions) / sizeof(test_positions[0]);
    
//     for (int i = 0; i < num_positions; i++) {
//         ESP_LOGI(TAG, "Moviendo a %d grados...", test_positions[i]);
//         ret = servo_simple_set_position(test_positions[i]);
//         if (ret != ESP_OK) {
//             ESP_LOGE(TAG, "Error moviendo a %d°: %s", test_positions[i], esp_err_to_name(ret));
//         }
        
//         // Verificar posición actual
//         uint16_t current_pos = readAngle_simple();
//         ESP_LOGI(TAG, "Posición actual reportada: %d° | Posición esperada: %d°", current_pos, test_positions[i]);
        
//         vTaskDelay(1500 / portTICK_PERIOD_MS);
//     }

//     // ===== PRUEBA 5: Prueba de precisión =====
//     ESP_LOGI(TAG, "===== PRUEBA 5: Prueba de precisión =====");
    
//     ESP_LOGI(TAG, "Probando precisión en incrementos de 1 grado...");
//     servo_simple_set_position(SERVO_POSITION_CENTER); // Posición base
//     vTaskDelay(1000 / portTICK_PERIOD_MS);
    
//     // Movimientos pequeños desde el centro
//     for (int offset = 1; offset <= 10; offset++) {
//         ESP_LOGI(TAG, "90 + %d = %d°", offset, 90 + offset);
//         servo_simple_set_position(90 + offset);
//         vTaskDelay(200 / portTICK_PERIOD_MS);
        
//         ESP_LOGI(TAG, "90 - %d = %d°", offset, 90 - offset);
//         servo_simple_set_position(90 - offset);
//         vTaskDelay(200 / portTICK_PERIOD_MS);
//     }

//     // Volver al centro
//     ESP_LOGI(TAG, "Volviendo al centro (90°)...");
//     servo_simple_set_position(SERVO_POSITION_CENTER);
//     vTaskDelay(1000 / portTICK_PERIOD_MS);

//     // ===== PRUEBA 6: Información del servo =====
//     ESP_LOGI(TAG, "===== PRUEBA 6: Información del servo =====");
    
//     ESP_LOGI(TAG, "Estado del servo:");
//     ESP_LOGI(TAG, "  - Habilitado: %s", servo_simple_is_enabled() ? "SI" : "NO");
//     ESP_LOGI(TAG, "  - Posición actual: %d°", readAngle_simple());

//     // ===== PRUEBA 7: Prueba de límites =====
//     ESP_LOGI(TAG, "===== PRUEBA 7: Prueba de límites =====");
    
//     ESP_LOGI(TAG, "Probando límites del servo...");
    
//     // Intentar posición fuera de rango (debería fallar)
//     ESP_LOGI(TAG, "Intentando posición 200° (fuera de rango)...");
//     ret = servo_simple_set_position(200);
//     if (ret != ESP_OK) {
//         ESP_LOGI(TAG, "Correcto: Error esperado para posición fuera de rango: %s", esp_err_to_name(ret));
//     } else {
//         ESP_LOGW(TAG, "Inesperado: Posición fuera de rango fue aceptada");
//     }

//     // Posiciones en los límites exactos
//     ESP_LOGI(TAG, "Probando límite inferior (0°)...");
//     servo_simple_set_position(0);
//     vTaskDelay(1000 / portTICK_PERIOD_MS);
    
//     ESP_LOGI(TAG, "Probando límite superior (180°)...");
//     servo_simple_set_position(180);
//     vTaskDelay(1000 / portTICK_PERIOD_MS);

//     // ===== FINALIZACIÓN =====
//     ESP_LOGI(TAG, "===== FINALIZANDO PRUEBAS =====");
    
//     // Posición final segura
//     ESP_LOGI(TAG, "Moviendo a posición final segura (90°)...");
//     servo_simple_set_position(90);
//     vTaskDelay(1000 / portTICK_PERIOD_MS);

//     // Deshabilitar servo
//     ESP_LOGI(TAG, "Deshabilitando servo...");
//     servo_simple_pause();

//     // Limpiar recursos
//     ESP_LOGI(TAG, "Liberando recursos...");
//     servo_simple_stop();

//     ESP_LOGI(TAG, "===== PRUEBAS COMPLETADAS =====");
//     ESP_LOGI(TAG, "Resultados:");
//     ESP_LOGI(TAG, "  - Si el servo se movió suavemente: ✅ SG90 funcionando correctamente");
//     ESP_LOGI(TAG, "  - Si hubo movimientos bruscos: ⚠️  Verificar alimentación y conexiones");
//     ESP_LOGI(TAG, "  - Si no se movió: ❌ Verificar:");
//     ESP_LOGI(TAG, "    * Conexión de señal (GPIO 14)");
//     ESP_LOGI(TAG, "    * Alimentación (5V y GND)");
//     ESP_LOGI(TAG, "    * Que el servo sea un SG90 genuine");

//     return 0;
// }