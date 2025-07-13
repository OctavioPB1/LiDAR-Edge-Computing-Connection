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

#include "i2c.h"
// Include the generic servo library for SG90
#include "Generic_servo/servo_generic.h"

static const char *TAG = "SG90_TEST";

// SG90 Servo Specifications
// - Operating Voltage: 4.8V~6V
// - Operating Speed: 0.1sec/60degree (4.8V), 0.08sec/60degree (6V)
// - Stall Torque: 1.8kg.cm (4.8V), 2.5kg.cm (6V)
// - Operating Angle: 0-180 degrees
// - Required Pulse: 500us-2500us (typical)
// - Frequency: 50Hz

// Custom configuration for SG90
const servo_config_t SG90_CONFIG = {
    .pin = GPIO_NUM_14,
    .type = SERVO_TYPE_POSITION,
    .min_pulse_us = 350,      // SG90 minimum pulse
    .max_pulse_us = 2000,     // SG90 maximum pulse
    .center_pulse_us = 1000,  // SG90 center pulse (90 degrees)
    .frequency_hz = 50,       // Standard servo frequency
    .min_angle = 0,           // Minimum angle
    .max_angle = 180,         // Maximum angle
    .speed = 50,              // Default speed (not used for position servos)
    .invert_direction = false // Normal direction
};

int app_main(void)
{
    servo_handle_t *sg90_servo = NULL;
    esp_err_t ret;

    ESP_LOGI(TAG, "======= PROGRAMA DE PRUEBA SERVO SG90 =======");
    ESP_LOGI(TAG, "ESP-IDF Version: %s", esp_get_idf_version());
    ESP_LOGI(TAG, "Servo: SG90 (Servo de posición 0-180°)");
    ESP_LOGI(TAG, "GPIO: 14");
    ESP_LOGI(TAG, "Alimentación: 5V");

    // Inicializar el servo SG90
    ESP_LOGI(TAG, "Inicializando servo SG90...");
    ret = servo_generic_init(&SG90_CONFIG, &sg90_servo);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error inicializando SG90: %s", esp_err_to_name(ret));
        return 1;
    }
    ESP_LOGI(TAG, "SG90 inicializado correctamente!");

    // Habilitar el servo
    ESP_LOGI(TAG, "Habilitando servo SG90...");
    ret = servo_generic_enable(sg90_servo);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error habilitando SG90: %s", esp_err_to_name(ret));
        servo_generic_deinit(sg90_servo);
        return 1;
    }
    ESP_LOGI(TAG, "SG90 habilitado - Listo para pruebas!");

    // Esperar un momento para estabilización
    vTaskDelay(1000 / portTICK_PERIOD_MS);

    // ===== PRUEBA 1: Posiciones básicas =====
    ESP_LOGI(TAG, "===== PRUEBA 1: Posiciones básicas =====");
    
    // Posición 0 grados
    ESP_LOGI(TAG, "Moviendo a 0 grados...");
    ret = servo_generic_set_position(sg90_servo, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error moviendo a 0°: %s", esp_err_to_name(ret));
    }
    vTaskDelay(1000 / portTICK_PERIOD_MS);

    // Posición 90 grados (centro)
    ESP_LOGI(TAG, "Moviendo a 90 grados (centro)...");
    ret = servo_generic_set_position(sg90_servo, 90);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error moviendo a 90°: %s", esp_err_to_name(ret));
    }
    vTaskDelay(1000 / portTICK_PERIOD_MS);

    // Posición 180 grados
    ESP_LOGI(TAG, "Moviendo a 180 grados...");
    ret = servo_generic_set_position(sg90_servo, 180);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error moviendo a 180°: %s", esp_err_to_name(ret));
    }
    vTaskDelay(1000 / portTICK_PERIOD_MS);

    // ===== PRUEBA 2: Barrido completo =====
    ESP_LOGI(TAG, "===== PRUEBA 2: Barrido completo (0-180-0) =====");
    
    // Barrido de 0 a 180 grados
    ESP_LOGI(TAG, "Barrido de 0 a 180 grados...");
    for (int angle = 0; angle <= 180; angle += 2) {
        ESP_LOGI(TAG, "Posición: %d°", angle);
        servo_generic_set_position(sg90_servo, angle);
        vTaskDelay(300 / portTICK_PERIOD_MS);
    }

    // Barrido de 180 a 0 grados
    ESP_LOGI(TAG, "Barrido de 180 a 0 grados...");
    for (int angle = 180; angle >= 0; angle -= 2) {
        ESP_LOGI(TAG, "Posición: %d°", angle);
        servo_generic_set_position(sg90_servo, angle);
        vTaskDelay(300 / portTICK_PERIOD_MS);
    }

    // // ===== PRUEBA 3: Movimiento suave =====
    // ESP_LOGI(TAG, "===== PRUEBA 3: Movimiento suave =====");
    
    // ESP_LOGI(TAG, "Movimiento suave de 0 a 180 grados (3 segundos)...");
    // servo_generic_move_smooth(sg90_servo, 180, 3000);
    
    // ESP_LOGI(TAG, "Movimiento suave de 180 a 0 grados (3 segundos)...");
    // servo_generic_move_smooth(sg90_servo, 0, 3000);

    // // ===== PRUEBA 4: Posiciones específicas =====
    // ESP_LOGI(TAG, "===== PRUEBA 4: Posiciones específicas =====");
    
    // uint16_t test_positions[] = {45, 135, 30, 150, 60, 120, 90};
    // int num_positions = sizeof(test_positions) / sizeof(test_positions[0]);
    
    // for (int i = 0; i < num_positions; i++) {
    //     ESP_LOGI(TAG, "Moviendo a %d grados...", test_positions[i]);
    //     servo_generic_set_position(sg90_servo, test_positions[i]);
        
    //     // Verificar posición actual
    //     uint16_t current_pos = servo_generic_get_position(sg90_servo);
    //     ESP_LOGI(TAG, "Posición actual reportada: %d°", current_pos);
        
    //     vTaskDelay(1500 / portTICK_PERIOD_MS);
    // }

    // // ===== PRUEBA 5: Prueba de precisión =====
    // ESP_LOGI(TAG, "===== PRUEBA 5: Prueba de precisión =====");
    
    // ESP_LOGI(TAG, "Probando precisión en incrementos de 1 grado...");
    // servo_generic_set_position(sg90_servo, 90); // Posición base
    // vTaskDelay(1000 / portTICK_PERIOD_MS);
    
    // // Movimientos pequeños desde el centro
    // for (int offset = 1; offset <= 10; offset++) {
    //     ESP_LOGI(TAG, "90 + %d = %d°", offset, 90 + offset);
    //     servo_generic_set_position(sg90_servo, 90 + offset);
    //     vTaskDelay(200 / portTICK_PERIOD_MS);
        
    //     ESP_LOGI(TAG, "90 - %d = %d°", offset, 90 - offset);
    //     servo_generic_set_position(sg90_servo, 90 - offset);
    //     vTaskDelay(200 / portTICK_PERIOD_MS);
    // }

    // // Volver al centro
    // ESP_LOGI(TAG, "Volviendo al centro (90°)...");
    // servo_generic_set_position(sg90_servo, 90);
    // vTaskDelay(1000 / portTICK_PERIOD_MS);

    // // ===== PRUEBA 6: Información del servo =====
    // ESP_LOGI(TAG, "===== PRUEBA 6: Información del servo =====");
    
    // ESP_LOGI(TAG, "Estado del servo:");
    // ESP_LOGI(TAG, "  - Habilitado: %s", servo_generic_is_enabled(sg90_servo) ? "SI" : "NO");
    // ESP_LOGI(TAG, "  - Posición actual: %d°", servo_generic_get_position(sg90_servo));
    // ESP_LOGI(TAG, "  - Pulso actual: %lu us", servo_generic_get_pulse(sg90_servo));
    // ESP_LOGI(TAG, "  - Velocidad configurada: %d%%", servo_generic_get_speed(sg90_servo));

    // // ===== PRUEBA 7: Prueba de límites =====
    // ESP_LOGI(TAG, "===== PRUEBA 7: Prueba de límites =====");
    
    // ESP_LOGI(TAG, "Probando límites del servo...");
    
    // // Intentar posición fuera de rango (debería fallar)
    // ESP_LOGI(TAG, "Intentando posición 200° (fuera de rango)...");
    // ret = servo_generic_set_position(sg90_servo, 200);
    // if (ret != ESP_OK) {
    //     ESP_LOGI(TAG, "Correcto: Error esperado para posición fuera de rango: %s", esp_err_to_name(ret));
    // } else {
    //     ESP_LOGW(TAG, "Inesperado: Posición fuera de rango fue aceptada");
    // }

    // // Posiciones en los límites exactos
    // ESP_LOGI(TAG, "Probando límite inferior (0°)...");
    // servo_generic_set_position(sg90_servo, 0);
    // vTaskDelay(1000 / portTICK_PERIOD_MS);
    
    // ESP_LOGI(TAG, "Probando límite superior (180°)...");
    // servo_generic_set_position(sg90_servo, 180);
    // vTaskDelay(1000 / portTICK_PERIOD_MS);

    // ===== FINALIZACIÓN =====
    ESP_LOGI(TAG, "===== FINALIZANDO PRUEBAS =====");
    
    // Posición final segura
    ESP_LOGI(TAG, "Moviendo a posición final segura (90°)...");
    servo_generic_set_position(sg90_servo, 90);
    vTaskDelay(1000 / portTICK_PERIOD_MS);

    // Deshabilitar servo
    ESP_LOGI(TAG, "Deshabilitando servo...");
    servo_generic_disable(sg90_servo);

    // Limpiar recursos
    ESP_LOGI(TAG, "Liberando recursos...");
    servo_generic_deinit(sg90_servo);

    ESP_LOGI(TAG, "===== PRUEBAS COMPLETADAS =====");
    ESP_LOGI(TAG, "Resultados:");
    ESP_LOGI(TAG, "  - Si el servo se movió suavemente: ✅ SG90 funcionando correctamente");
    ESP_LOGI(TAG, "  - Si hubo movimientos bruscos: ⚠️  Verificar alimentación y conexiones");
    ESP_LOGI(TAG, "  - Si no se movió: ❌ Verificar:");
    ESP_LOGI(TAG, "    * Conexión de señal (GPIO 14)");
    ESP_LOGI(TAG, "    * Alimentación (5V y GND)");
    ESP_LOGI(TAG, "    * Que el servo sea un SG90 genuine");

    return 0;
}