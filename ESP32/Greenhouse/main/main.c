#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_wifi.h"

#include "wifi.h"
#include "mqtt.h"
#include "sensors.h"
#include "relays.h"
#include "ventilation.h"
#include "cooling.h"

static const char *TAG = "GREENHOUSE";

// Poll interval for sensors in milliseconds
#define SENSOR_POLL_INTERVAL 10000

void app_main(void)
{
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    
    ESP_LOGI(TAG, "Initializing system...");

    // Initialize hardware peripherals and sensors
    init_sensors();
    init_relays();
    
    // Connect to WiFi
    wifi_init_sta();
    
    // Setup MQTT client
    mqtt_init();
    
    // Initialize ventilation and cooling systems
    ventilation_init();
    cooling_init();
    
    // Restore saved system states
    load_system_state();

    TickType_t last_sensor_read = 0;
    
    // Main loop
    while (1) {
        // Check WiFi and MQTT connection status, reconnect if needed
        if (!wifi_is_connected()) {
            wifi_reconnect();
        }
        
        if (wifi_is_connected() && !mqtt_is_connected()) {
            mqtt_reconnect();
        }
        
        // Process MQTT messages
        mqtt_loop();
        
        // Read sensors periodically
        TickType_t now = xTaskGetTickCount();
        if ((now - last_sensor_read) >= (SENSOR_POLL_INTERVAL / portTICK_PERIOD_MS)) {
            last_sensor_read = now;
            
            // Read temperature and humidity from SHT20
            read_sht20_data();
            
            // Handle automatic control systems
            ventilation_control_auto();
            cooling_control_auto();
        }
        
        // Check ventilation status and update if needed
        ventilation_check_progress();
        
        // Short delay to avoid consuming too much CPU
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

// Load system state from NVS
void load_system_state(void)
{
    relays_load_state();
    ventilation_load_state();
    cooling_load_state();
} 