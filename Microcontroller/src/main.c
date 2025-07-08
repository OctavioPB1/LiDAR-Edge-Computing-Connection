// #include "esp_log.h"
// #include "cyclops_core.h"
// #include "freertos/FreeRTOS.h"
// #include "freertos/task.h"
// #include "instruction_buffer.h"
// #include <stdio.h>
// #include <string.h>
// #include "mqtt_handler.h"
// static const char *TAG = "MAIN";

// void app_main(void)
// {
//         esp_err_t err = ESP_FAIL;
//         err = system_init();
//         if (err != ESP_OK)
//         {
//                 ESP_LOGE(TAG, "FATAL ERROR");
//         }
//         else
//         {
//                 err = createTasks();
//                 if (err != ESP_OK)
//                 {
//                         ESP_LOGE(TAG, "FATAL ERROR");
//                 }
//         }

//         ESP_LOGI(TAG, "ALL DONE");

//         TESTING SAVING INST
//         char inst1[] = "TESTING SEMDING MESSAGE";

//             while (1)
//         {
//                 err = sendInfoMesage(TAG,inst1);
//                 if (err != ESP_OK)
//                 {
//                         ESP_LOGW(TAG, "MESSAGE SEND FAIL: %s", esp_err_to_name(err));
//                 } else {
//                         ESP_LOGI(TAG,"MESSAGE SEND");
//                 }
//                 vTaskDelay(1500 / portTICK_PERIOD_MS);
//         }

// }

#include "esp_log.h"
#include "cyclops_core.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "LiDAR_Library/mapping.h"
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