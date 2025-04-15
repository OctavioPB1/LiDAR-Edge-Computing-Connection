#include "client_mode.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_netif.h"

#define WIFI_SSID "wifilora"
#define WIFI_PASS "12345678"

static const char *TAG = "client mode";

esp_err_t client_mode_init() {
    esp_err_t err;

    err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
        if (err != ESP_OK){
            ESP_LOGE(TAG, "Error initializing NVS: %s", esp_err_to_name(err));
            return err;
        }
    }
    

    err = esp_netif_init();
    if(err != ESP_OK){
        ESP_LOGE(TAG, "Error initializing NETIF: %s", esp_err_to_name(err));
        return err;
    }
    err = esp_event_loop_create_default();
    if(err != ESP_OK){
        ESP_LOGE(TAG, "Error initializing Event LOOP: %s", esp_err_to_name(err));
        return err;
    }
    esp_netif_create_default_wifi_sta(); // crea interfaz por defecto (usa DHCP)

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    err = esp_wifi_init(&cfg);
    if(err != ESP_OK){
        ESP_LOGE(TAG, "Error allocating WiFi resources: %s", esp_err_to_name(err));
        return err;
    }

    wifi_config_t wifi_config = {
        .sta = {
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
        },
    };
    strcpy((char*)wifi_config.sta.ssid, WIFI_SSID);
    strcpy((char*)wifi_config.sta.password, WIFI_PASS);

    err = esp_wifi_set_mode(WIFI_MODE_STA);
    if(err != ESP_OK){
        ESP_LOGE(TAG, "Error setting station mode: %s", esp_err_to_name(err));
        return err;
    }
    err = esp_wifi_set_config(WIFI_IF_STA, &wifi_config);
    if(err != ESP_OK){
        ESP_LOGE(TAG, "Error setting the configuration: %s", esp_err_to_name(err));
        return err;
    }
    err = esp_wifi_start();
    if(err != ESP_OK){
        ESP_LOGE(TAG, "Error starting: %s", esp_err_to_name(err));
        return err;
    }

    ESP_LOGI(TAG, "Conectando a WiFi...");
    err = esp_wifi_connect();
    if(err != ESP_OK){
        ESP_LOGE(TAG, "Error connecting: %s", esp_err_to_name(err));
        return err;
    }
    return ESP_OK;
}

esp_err_t check_assigned_ip(){
    esp_netif_ip_info_t ip_info;
    esp_err_t err = esp_netif_get_ip_info(esp_netif_get_handle_from_ifkey("WIFI_STA_DEF"), &ip_info);
    if(err != ESP_OK){
        ESP_LOGE(TAG, "Error getting IP: %s", esp_err_to_name(err));
        return err;
    }
    ESP_LOGI(TAG, "IP asignada por DHCP: " IPSTR, IP2STR(&ip_info.ip));
    return ESP_OK;
}
