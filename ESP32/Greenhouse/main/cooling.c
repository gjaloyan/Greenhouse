#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "cJSON.h"

#include "cooling.h"
#include "relays.h"
#include "sensors.h"
#include "mqtt.h"
#include "ventilation.h"
#include "greenhouse_config.h"

static const char *TAG = "COOLING";

// Cooling system variables
static const int COOLING_WATER_PUMP_RELAY = 7;  // Relay number for water pump

// Ventilator relays start from this number
static const int COOLING_VENTILATOR_FIRST_RELAY = 3;

static float cooling_target_temperature = 28.0;
static float cooling_emergency_off_temperature = 15.0;
static bool cooling_control_auto_state = false;
static bool cooling_system_active = false;
static bool cooling_water_pump_status = false;
static int cooling_ventilators_count = MAX_COOLING_VENTILATORS;
static bool cooling_ventilators_status[MAX_COOLING_VENTILATORS] = {false};

void cooling_init(void)
{
    ESP_LOGI(TAG, "Initializing cooling system");
    
    // Ensure cooling system is off at startup
    cooling_system_stop();
}

bool cooling_is_active(void)
{
    return cooling_system_active;
}

void cooling_system_start(int percent)
{
    // First check if ventilation is open, and close it if needed
    ventilation_control_open_percent(0);
    
    // Wait for ventilation to fully close - this is a blocking call with timeout
    unsigned long start_wait_time = esp_timer_get_time() / 1000; // convert to ms
    const unsigned long max_wait_time = 60000; // Maximum wait: 60 seconds
    
    while (esp_timer_get_time() / 1000 - start_wait_time < max_wait_time) {
        // Wait briefly
        vTaskDelay(500 / portTICK_PERIOD_MS);
        
        // Keep MQTT running while waiting
        mqtt_loop();
        
        // Check if ventilation has completed closing
        // We don't have direct access to ventilation internal state, so we can only wait for timeout
    }
    
    ESP_LOGI(TAG, "Starting cooling system at %d%%", percent);
    
    // Normalize percent value
    if (percent < 0) percent = 0;
    if (percent > 100) percent = 100;
    
    // First stop all ventilators
    cooling_system_stop_ventilators();
    
    // Turn on cooling fans based on percentage
    if (percent >= 25) {
        cooling_start_ventilator(1);
        cooling_system_active = true;
    }
    
    if (percent >= 50) {
        cooling_start_ventilator(1);
        cooling_start_ventilator(2);
        cooling_system_active = true;
    }
    
    if (percent >= 75) {
        cooling_start_ventilator(1);
        cooling_start_ventilator(2);
        cooling_start_ventilator(3);
        cooling_system_active = true;
    }
    
    if (percent >= 100) {
        cooling_start_ventilator(1);
        cooling_start_ventilator(2);
        cooling_start_ventilator(3);
        cooling_start_ventilator(4);
        cooling_system_active = true;
    }
    
    // Update water pump status if needed
    if (percent > 0 && !cooling_water_pump_status) {
        cooling_system_start_water_pump();
    } else if (percent == 0 && cooling_water_pump_status) {
        cooling_system_stop_water_pump();
    }
    
    // Save settings and update status
    cooling_save_settings();
    cooling_publish_status();
}

void cooling_system_start_water_pump(void)
{
    if (relay_turn_on(COOLING_WATER_PUMP_RELAY)) {
        cooling_water_pump_status = true;
        ESP_LOGI(TAG, "Cooling water pump started");
    } else {
        ESP_LOGE(TAG, "Failed to start cooling water pump");
    }
    
    cooling_save_settings();
    cooling_publish_status();
}

void cooling_system_stop(void)
{
    if (cooling_system_active) {
        cooling_system_stop_ventilators();
        cooling_system_stop_water_pump();
        cooling_system_active = false;
        
        ESP_LOGI(TAG, "Cooling system stopped");
        
        cooling_save_settings();
        cooling_publish_status();
    }
}

void cooling_system_stop_ventilators(void)
{
    for (int i = 0; i < cooling_ventilators_count; i++) {
        cooling_stop_ventilator(i + 1);
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
    
    // Ensure all flags are off
    for (int i = 0; i < cooling_ventilators_count; i++) {
        cooling_ventilators_status[i] = false;
    }
    
    cooling_system_active = false;
    cooling_save_settings();
    cooling_publish_status();
}

void cooling_system_stop_water_pump(void)
{
    if (relay_turn_off(COOLING_WATER_PUMP_RELAY)) {
        cooling_water_pump_status = false;
        ESP_LOGI(TAG, "Cooling water pump stopped");
    } else {
        ESP_LOGE(TAG, "Failed to stop cooling water pump");
    }
    
    cooling_save_settings();
    cooling_publish_status();
}

void cooling_start_ventilator(int ventilator_num)
{
    if (ventilator_num < 1 || ventilator_num > cooling_ventilators_count) {
        ESP_LOGE(TAG, "Invalid ventilator number: %d", ventilator_num);
        return;
    }
    
    // Calculate the relay number
    int relay_num = COOLING_VENTILATOR_FIRST_RELAY + ventilator_num - 1;
    
    if (relay_turn_on(relay_num)) {
        cooling_ventilators_status[ventilator_num - 1] = true;
        ESP_LOGI(TAG, "Ventilator %d turned on", ventilator_num);
        
        // Save ventilator state
        char key[8];
        sprintf(key, "vent%d", ventilator_num);
        
        nvs_handle_t nvs_handle;
        if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs_handle) == ESP_OK) {
            nvs_set_u8(nvs_handle, key, 1);
            nvs_commit(nvs_handle);
            nvs_close(nvs_handle);
        }
        
        cooling_publish_status();
    } else {
        ESP_LOGE(TAG, "Ventilator %d turn on error", ventilator_num);
    }
}

void cooling_stop_ventilator(int ventilator_num)
{
    if (ventilator_num < 1 || ventilator_num > cooling_ventilators_count) {
        ESP_LOGE(TAG, "Invalid ventilator number: %d", ventilator_num);
        return;
    }
    
    // Calculate the relay number
    int relay_num = COOLING_VENTILATOR_FIRST_RELAY + ventilator_num - 1;
    
    if (relay_turn_off(relay_num)) {
        cooling_ventilators_status[ventilator_num - 1] = false;
        ESP_LOGI(TAG, "Ventilator %d turned off", ventilator_num);
        
        // Save ventilator state
        char key[8];
        sprintf(key, "vent%d", ventilator_num);
        
        nvs_handle_t nvs_handle;
        if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs_handle) == ESP_OK) {
            nvs_set_u8(nvs_handle, key, 0);
            nvs_commit(nvs_handle);
            nvs_close(nvs_handle);
        }
        
        cooling_publish_status();
    } else {
        ESP_LOGE(TAG, "Ventilator %d turn off error", ventilator_num);
    }
}

void cooling_set_auto_control(bool auto_state)
{
    cooling_control_auto_state = auto_state;
    ESP_LOGI(TAG, "Cooling auto control set to %s", auto_state ? "AUTO" : "MANUAL");
    
    // If switching to auto, run auto control immediately
    if (auto_state) {
        cooling_control_auto();
    }
    
    cooling_save_settings();
    cooling_publish_status();
}

void cooling_control_auto(void)
{
    if (cooling_control_auto_state) {
        float temperature = 0.0;
        float humidity = 0.0;
        
        // Read current temperature
        if (read_sht20_values(&temperature, &humidity)) {
            if (temperature > cooling_target_temperature && !cooling_system_active) {
                cooling_system_start(25);
            } else if (temperature > cooling_target_temperature + 2) {
                cooling_system_start(50);
            } else if (temperature > cooling_target_temperature + 4) {
                cooling_system_start(75);
            } else if (temperature > cooling_target_temperature + 6) {
                cooling_system_start(100);
            } else if (temperature < cooling_target_temperature) {
                cooling_system_stop();
            } else if (cooling_emergency_off_temperature > 0 && temperature < cooling_emergency_off_temperature) {
                cooling_system_active = false;
                cooling_system_stop();
            }
        } else {
            ESP_LOGW(TAG, "Failed to read temperature for auto cooling");
        }
    }
}

void cooling_set_setpoints(float target_temp, float emergency_off_temp, bool auto_control)
{
    if (target_temp > 0) {
        cooling_target_temperature = target_temp;
    }
    
    cooling_emergency_off_temperature = emergency_off_temp;
    cooling_control_auto_state = auto_control;
    
    ESP_LOGI(TAG, "Cooling setpoints updated: Target=%.1f°C, Emergency=%.1f°C, Auto=%s",
             cooling_target_temperature, cooling_emergency_off_temperature,
             cooling_control_auto_state ? "ON" : "OFF");
    
    cooling_save_settings();
    cooling_publish_setpoints();
    
    // If auto control is enabled, run it immediately
    if (auto_control) {
        cooling_control_auto();
    }
}

void cooling_load_state(void)
{
    nvs_handle_t nvs_handle;
    esp_err_t err;
    
    err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error opening NVS namespace: %s", esp_err_to_name(err));
        return;
    }
    
    // Load target temperature
    float target_temp;
    err = nvs_get_float(nvs_handle, "cool_target", &target_temp);
    if (err == ESP_OK) {
        cooling_target_temperature = target_temp;
    } else if (err != ESP_ERR_NVS_NOT_FOUND) {
        ESP_LOGE(TAG, "Error reading cool_target: %s", esp_err_to_name(err));
    }
    
    // Load emergency off temperature
    float emergency_temp;
    err = nvs_get_float(nvs_handle, "cool_emerg", &emergency_temp);
    if (err == ESP_OK) {
        cooling_emergency_off_temperature = emergency_temp;
    } else if (err != ESP_ERR_NVS_NOT_FOUND) {
        ESP_LOGE(TAG, "Error reading cool_emerg: %s", esp_err_to_name(err));
    }
    
    // Load auto control state
    uint8_t auto_control;
    err = nvs_get_u8(nvs_handle, "cool_auto", &auto_control);
    if (err == ESP_OK) {
        cooling_control_auto_state = (auto_control == 1);
    } else if (err != ESP_ERR_NVS_NOT_FOUND) {
        ESP_LOGE(TAG, "Error reading cool_auto: %s", esp_err_to_name(err));
    }
    
    // Load system active state
    uint8_t system_active;
    err = nvs_get_u8(nvs_handle, "cool_active", &system_active);
    if (err == ESP_OK) {
        cooling_system_active = (system_active == 1);
    } else if (err != ESP_ERR_NVS_NOT_FOUND) {
        ESP_LOGE(TAG, "Error reading cool_active: %s", esp_err_to_name(err));
    }
    
    // Load water pump status
    uint8_t water_pump;
    err = nvs_get_u8(nvs_handle, "water_pump", &water_pump);
    if (err == ESP_OK) {
        cooling_water_pump_status = (water_pump == 1);
    } else if (err != ESP_ERR_NVS_NOT_FOUND) {
        ESP_LOGE(TAG, "Error reading water_pump: %s", esp_err_to_name(err));
    }
    
    // Load ventilator count
    int32_t vent_count;
    err = nvs_get_i32(nvs_handle, "vent_count", &vent_count);
    if (err == ESP_OK && vent_count > 0 && vent_count <= MAX_COOLING_VENTILATORS) {
        cooling_ventilators_count = vent_count;
    } else if (err != ESP_ERR_NVS_NOT_FOUND) {
        ESP_LOGE(TAG, "Error reading vent_count: %s", esp_err_to_name(err));
    }
    
    // Load ventilator statuses
    for (int i = 0; i < cooling_ventilators_count; i++) {
        char key[8];
        sprintf(key, "vent%d", i + 1);
        
        uint8_t vent_status;
        err = nvs_get_u8(nvs_handle, key, &vent_status);
        
        if (err == ESP_OK) {
            cooling_ventilators_status[i] = (vent_status == 1);
            
            // Apply saved state to hardware
            if (cooling_ventilators_status[i]) {
                cooling_start_ventilator(i + 1);
            } else {
                cooling_stop_ventilator(i + 1);
            }
        } else if (err != ESP_ERR_NVS_NOT_FOUND) {
            ESP_LOGE(TAG, "Error reading ventilator %d status: %s", i + 1, esp_err_to_name(err));
        }
    }
    
    nvs_close(nvs_handle);
    
    ESP_LOGI(TAG, "Cooling state loaded: Active=%s, Auto=%s, Target=%.1f°C", 
             cooling_system_active ? "YES" : "NO",
             cooling_control_auto_state ? "ON" : "OFF", 
             cooling_target_temperature);
}

void cooling_save_settings(void)
{
    nvs_handle_t nvs_handle;
    esp_err_t err;
    
    err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error opening NVS namespace: %s", esp_err_to_name(err));
        return;
    }
    
    // Save target temperature
    err = nvs_set_float(nvs_handle, "cool_target", cooling_target_temperature);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error saving cool_target: %s", esp_err_to_name(err));
    }
    
    // Save emergency off temperature
    err = nvs_set_float(nvs_handle, "cool_emerg", cooling_emergency_off_temperature);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error saving cool_emerg: %s", esp_err_to_name(err));
    }
    
    // Save auto control state
    err = nvs_set_u8(nvs_handle, "cool_auto", cooling_control_auto_state ? 1 : 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error saving cool_auto: %s", esp_err_to_name(err));
    }
    
    // Save system active state
    err = nvs_set_u8(nvs_handle, "cool_active", cooling_system_active ? 1 : 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error saving cool_active: %s", esp_err_to_name(err));
    }
    
    // Save water pump status
    err = nvs_set_u8(nvs_handle, "water_pump", cooling_water_pump_status ? 1 : 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error saving water_pump: %s", esp_err_to_name(err));
    }
    
    // Save ventilator count
    err = nvs_set_i32(nvs_handle, "vent_count", cooling_ventilators_count);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error saving vent_count: %s", esp_err_to_name(err));
    }
    
    // Save ventilator statuses
    for (int i = 0; i < cooling_ventilators_count; i++) {
        char key[8];
        sprintf(key, "vent%d", i + 1);
        
        err = nvs_set_u8(nvs_handle, key, cooling_ventilators_status[i] ? 1 : 0);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Error saving ventilator %d status: %s", i + 1, esp_err_to_name(err));
        }
    }
    
    err = nvs_commit(nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error committing NVS changes: %s", esp_err_to_name(err));
    }
    
    nvs_close(nvs_handle);
}

void cooling_publish_status(void)
{
    cJSON *root = cJSON_CreateObject();
    
    // Add basic status
    cJSON_AddBoolToObject(root, "status", cooling_system_active);
    cJSON_AddBoolToObject(root, "water_pump", cooling_water_pump_status);
    cJSON_AddBoolToObject(root, "cooling_control_auto_state", cooling_control_auto_state);
    
    // Add ventilator statuses as an array
    cJSON *ventilators = cJSON_CreateArray();
    for (int i = 0; i < cooling_ventilators_count; i++) {
        cJSON_AddItemToArray(ventilators, cJSON_CreateBool(cooling_ventilators_status[i]));
    }
    cJSON_AddItemToObject(root, "ventilators", ventilators);
    
    // Add greenhouse identification
    cJSON_AddStringToObject(root, "client_id", MQTT_CLIENT_ID);
    cJSON_AddStringToObject(root, "greenhouse_id", GREENHOUSE_ID);
    
    // Convert to string and publish
    char *data = cJSON_Print(root);
    mqtt_publish(TOPIC_COOLING_STATUS, data);
    
    free(data);
    cJSON_Delete(root);
    
    ESP_LOGI(TAG, "Published cooling status: Active=%s", cooling_system_active ? "YES" : "NO");
}

void cooling_publish_setpoints(void)
{
    cJSON *root = cJSON_CreateObject();
    
    cJSON_AddNumberToObject(root, "target_temperature", cooling_target_temperature);
    cJSON_AddNumberToObject(root, "emergency_off_temperature", cooling_emergency_off_temperature);
    cJSON_AddBoolToObject(root, "control_auto_state", cooling_control_auto_state);
    cJSON_AddStringToObject(root, "client_id", MQTT_CLIENT_ID);
    cJSON_AddStringToObject(root, "greenhouse_id", GREENHOUSE_ID);
    
    char *data = cJSON_Print(root);
    mqtt_publish(TOPIC_COOLING_SETPOINTS, data);
    
    free(data);
    cJSON_Delete(root);
} 