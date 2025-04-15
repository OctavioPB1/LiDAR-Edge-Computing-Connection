#ifndef CLIENT_MODE_H
#define CLIENT_MODE_H

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include <portmacro.h>

esp_err_t client_mode_init();
esp_err_t check_assigned_ip();

#endif
