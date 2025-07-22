#include "esp_log.h"
#include "cyclops_core.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
//#include "mapping.h"
#include "LiDAR_Library/mapping.h"
#include "LiDAR_Library/mapping_example.c"
#include "battery.h"
#include "motors.h"
#include "mqtt_handler.h"

#include "esp_timer.h"
#include "heap_trace_helper.h"

static const char *MAIN_TAG = "MAIN";

int app_main(void)
{
        esp_err_t err;

        heap_trace_start(HEAP_TRACE_ALL);

        ESP_LOGI(MAIN_TAG, "Iniciando Sistemas...");
        err = system_init();
        if (err != ESP_OK)
        {
                ESP_LOGE(MAIN_TAG, "Error Iniciando Sistemas:  %s", esp_err_to_name(err));
                return 1;
        }
        ESP_LOGI(MAIN_TAG, "Sistemas Iniciado!");
        // GENERO PUNTO DE RETORNO

        ESP_LOGI(MAIN_TAG, "Iniciando Tareas...");
        err = createTasks();
        if (err != ESP_OK)
        {
                ESP_LOGE(MAIN_TAG, "Error Iniciando Tareas:  %s", esp_err_to_name(err));
                return 1;
        }
        ESP_LOGI(MAIN_TAG, "Tareas Iniciadas!");
        
        // Tu bucle principal original (ejemplo)
        /*
        uint64_t time_now = esp_timer_get_time();
        while(1){
                saveInstruction("Forward");
                vTaskDelay(50 / portTICK_PERIOD_MS);

                if ( (esp_timer_get_time() - time_now) > 120000000 ){
                        ESP_LOGI(MAIN_TAG, "Pasó 1 minuto $$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$");
                        heap_caps_print_heap_info(MALLOC_CAP_8BIT);
                }
        }*/

        return 0;
}