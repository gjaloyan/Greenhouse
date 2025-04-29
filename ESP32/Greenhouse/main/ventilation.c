#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "cJSON.h"

#include "ventilation.h"
#include "relays.h"
#include "sensors.h"
#include "mqtt.h"
#include "cooling.h"
#include "greenhouse_config.h"

static const char *TAG = "VENTILATION";

// Ventilation system variables
static const int VENTILATION_RELAY_OPEN = 2;  // Relay number for opening ventilation
static const int VENTILATION_RELAY_CLOSE = 3; // Relay number for closing ventilation

static char ventilation_status[32] = "stopped";
static float ventilation_temperature_setpoint = 26.0;
static bool ventilation_control_auto_state = false;
static unsigned long ventilation_open_coefficient = 500;
static unsigned long ventilation_start_time = 0;
static unsigned long ventilation_open_duration = 0;
static int ventilation_percent_target = 0;
static bool ventilation_in_progress = false;
static int current_ventilation_percent = 0;
static int ventilation_wind_speed_setpoint = 0;
static float ventilation_emergency_off_temperature = 0.0;

void ventilation_init(void)
{
    ESP_LOGI(TAG, "Initializing ventilation system");
    
    // Stop any ventilation movement at startup
    ventilation_control_stop();
}

void ventilation_control_open(void)
{
    ESP_LOGI(TAG, "Starting ventilation opening");
    
    // Check if cooling system is active, if so disable it first
    if (cooling_is_active()) {
        ESP_LOGI(TAG, "Cooling system is active, turning it off");
        cooling_system_stop();
    }
    
    if (relay_turn_on(VENTILATION_RELAY_OPEN)) {
        strcpy(ventilation_status, "opening");
    } else {
        strcpy(ventilation_status, "opening_error");
    }
}

void ventilation_control_close(void)
{
    ESP_LOGI(TAG, "Starting ventilation closing");
    
    if (relay_turn_on(VENTILATION_RELAY_CLOSE)) {
        strcpy(ventilation_status, "closing");
    } else {
        strcpy(ventilation_status, "closing_error");
    }
}

void ventilation_control_stop(void)
{
    ESP_LOGI(TAG, "Stopping ventilation movement");
    
    relay_turn_off(VENTILATION_RELAY_OPEN);
    relay_turn_off(VENTILATION_RELAY_CLOSE);
    
    if (!relay_get_status(VENTILATION_RELAY_OPEN) && !relay_get_status(VENTILATION_RELAY_CLOSE)) {
        strcpy(ventilation_status, "stopped");
        current_ventilation_percent = ventilation_percent_target;
    } else {
        strcpy(ventilation_status, "stopped_error");
    }
    
    ventilation_publish_status();
    ventilation_save_settings();
}

void ventilation_control_open_percent(int target_percent)
{
    // Check if cooling system is active, if so disable it first
    if (cooling_is_active()) {
        ESP_LOGI(TAG, "Cooling system is active, turning it off");
        cooling_system_stop();
    }
    
    // Validate input range
    if (target_percent < 0) target_percent = 0;
    if (target_percent > 100) target_percent = 100;
    
    // Calculate the relative movement needed
    int relative_change = target_percent - current_ventilation_percent;
    
    if (relative_change == 0) {
        // No change needed
        ESP_LOGI(TAG, "Ventilation already at %d%%", current_ventilation_percent);
        ventilation_publish_status();
        return;
    }
    
    // Calculate duration based on the absolute value of relative change
    int ventilation_open_interval = abs(relative_change) * ventilation_open_coefficient;
    
    ESP_LOGI(TAG, "Current position: %d%%, Target: %d%%, Relative change: %d%%", 
             current_ventilation_percent, target_percent, relative_change);
    
    if (relative_change < 0) {
        ESP_LOGI(TAG, "Ventilation closing from %d%% to %d%% (Moving %d%%)",
                current_ventilation_percent, target_percent, abs(relative_change));
        
        strcpy(ventilation_status, "closing");
        
        // Start closing motor
        ventilation_control_close();
        
        // Set up variables for non-blocking operation
        ventilation_start_time = esp_timer_get_time() / 1000; // convert to ms
        ventilation_open_duration = ventilation_open_interval;
        ventilation_percent_target = target_percent;
        ventilation_in_progress = true;
    } else {
        ESP_LOGI(TAG, "Ventilation opening from %d%% to %d%% (Moving %d%%)",
                current_ventilation_percent, target_percent, abs(relative_change));
        
        strcpy(ventilation_status, "opening");
        
        // Start opening motor
        ventilation_control_open();
        
        // Set up variables for non-blocking operation
        ventilation_start_time = esp_timer_get_time() / 1000; // convert to ms
        ventilation_open_duration = ventilation_open_interval;
        ventilation_percent_target = target_percent;
        ventilation_in_progress = true;
    }
}

void ventilation_check_progress(void)
{
    if (ventilation_in_progress) {
        unsigned long now = esp_timer_get_time() / 1000; // convert to ms
        
        // Check for completion
        if (now - ventilation_start_time >= ventilation_open_duration) {
            ventilation_control_stop();
            ventilation_in_progress = false;
            
            ESP_LOGI(TAG, "Ventilation movement complete. Now at %d%%", current_ventilation_percent);
        }
        
        // Check for wind speed exceeding limit
        int wind_speed = read_wind_speed();
        if (wind_speed > ventilation_wind_speed_setpoint && ventilation_wind_speed_setpoint > 0) {
            ventilation_control_stop();
            ventilation_in_progress = false;
            
            ESP_LOGW(TAG, "Wind speed %d exceeds limit %d. Ventilation stopped.", 
                     wind_speed, ventilation_wind_speed_setpoint);
        }
    }
}

void ventilation_control_auto(void)
{
    if (ventilation_control_auto_state) {
        float temperature = 0.0;
        float humidity = 0.0;
        
        // Read current temperature
        if (read_sht20_values(&temperature, &humidity)) {
            float plus_2 = ventilation_temperature_setpoint + 2.0f;
            float plus_4 = ventilation_temperature_setpoint + 4.0f;
            float plus_6 = ventilation_temperature_setpoint + 6.0f;
            
            // Only make changes if ventilation is not currently moving
            if (strcmp(ventilation_status, "stopped") == 0) {
                if (temperature < ventilation_temperature_setpoint || 
                    (ventilation_emergency_off_temperature > 0 && temperature < ventilation_emergency_off_temperature)) {
                    ventilation_control_open_percent(0);
                } else if (temperature > ventilation_temperature_setpoint && temperature < plus_2) {
                    ventilation_control_open_percent(25);
                } else if (temperature >= plus_2 && temperature < plus_4) {
                    ventilation_control_open_percent(50);
                } else if (temperature >= plus_4 && temperature < plus_6) {
                    ventilation_control_open_percent(75);
                } else if (temperature >= plus_6) {
                    ventilation_control_open_percent(100);
                }
            }
        } else {
            ESP_LOGW(TAG, "Failed to read temperature for auto ventilation");
        }
    }
}

void ventilation_set_auto_control(bool auto_state)
{
    ventilation_control_auto_state = auto_state;
    ESP_LOGI(TAG, "Ventilation auto control set to %s", auto_state ? "AUTO" : "MANUAL");
    ventilation_publish_status();
}

void ventilation_set_setpoints(float temperature, int coefficient, int wind_speed, float emergency_off_temp)
{
    if (temperature > 0) {
        ventilation_temperature_setpoint = temperature;
    }
    
    if (coefficient > 0) {
        ventilation_open_coefficient = coefficient;
    }
    
    if (wind_speed >= 0) {
        ventilation_wind_speed_setpoint = wind_speed;
    }
    
    ventilation_emergency_off_temperature = emergency_off_temp;
    
    ESP_LOGI(TAG, "Ventilation setpoints updated: Temp=%.1f°C, Coef=%lu, Wind=%d, Emergency=%.1f°C",
             ventilation_temperature_setpoint, ventilation_open_coefficient, 
             ventilation_wind_speed_setpoint, ventilation_emergency_off_temperature);
    
    ventilation_save_settings();
    ventilation_publish_setpoints();
}

void ventilation_load_state(void)
{
    nvs_handle_t nvs_handle;
    esp_err_t err;
    
    err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error opening NVS namespace: %s", esp_err_to_name(err));
        return;
    }
    
    // Load temperature setpoint
    float temp_setpoint;
    err = nvs_get_float(nvs_handle, "temp_setpoint", &temp_setpoint);
    if (err == ESP_OK) {
        ventilation_temperature_setpoint = temp_setpoint;
    } else if (err != ESP_ERR_NVS_NOT_FOUND) {
        ESP_LOGE(TAG, "Error reading temp_setpoint: %s", esp_err_to_name(err));
    }
    
    // Load ventilation status
    char status[32];
    size_t status_size = sizeof(status);
    err = nvs_get_str(nvs_handle, "vent_status", status, &status_size);
    if (err == ESP_OK) {
        strcpy(ventilation_status, status);
    } else if (err != ESP_ERR_NVS_NOT_FOUND) {
        ESP_LOGE(TAG, "Error reading vent_status: %s", esp_err_to_name(err));
    }
    
    // Load current percentage
    int32_t percent;
    err = nvs_get_i32(nvs_handle, "vent_percent", &percent);
    if (err == ESP_OK) {
        current_ventilation_percent = percent;
    } else if (err != ESP_ERR_NVS_NOT_FOUND) {
        ESP_LOGE(TAG, "Error reading vent_percent: %s", esp_err_to_name(err));
    }
    
    // Load auto control state
    uint8_t auto_control;
    err = nvs_get_u8(nvs_handle, "auto_control", &auto_control);
    if (err == ESP_OK) {
        ventilation_control_auto_state = (auto_control == 1);
    } else if (err != ESP_ERR_NVS_NOT_FOUND) {
        ESP_LOGE(TAG, "Error reading auto_control: %s", esp_err_to_name(err));
    }
    
    // Load open coefficient
    uint32_t coefficient;
    err = nvs_get_u32(nvs_handle, "vent_coefficient", &coefficient);
    if (err == ESP_OK) {
        ventilation_open_coefficient = coefficient;
    } else if (err != ESP_ERR_NVS_NOT_FOUND) {
        ESP_LOGE(TAG, "Error reading vent_coefficient: %s", esp_err_to_name(err));
    }
    
    // Load wind speed setpoint
    int32_t wind_speed;
    err = nvs_get_i32(nvs_handle, "wind_speed", &wind_speed);
    if (err == ESP_OK) {
        ventilation_wind_speed_setpoint = wind_speed;
    } else if (err != ESP_ERR_NVS_NOT_FOUND) {
        ESP_LOGE(TAG, "Error reading wind_speed: %s", esp_err_to_name(err));
    }
    
    // Load emergency off temperature
    float emergency_temp;
    err = nvs_get_float(nvs_handle, "vent_emerg_temp", &emergency_temp);
    if (err == ESP_OK) {
        ventilation_emergency_off_temperature = emergency_temp;
    } else if (err != ESP_ERR_NVS_NOT_FOUND) {
        ESP_LOGE(TAG, "Error reading vent_emerg_temp: %s", esp_err_to_name(err));
    }
    
    nvs_close(nvs_handle);
    
    ESP_LOGI(TAG, "Ventilation state loaded: Status=%s, Current=%d%%, Auto=%s, Temp=%.1f°C", 
             ventilation_status, current_ventilation_percent, 
             ventilation_control_auto_state ? "ON" : "OFF", ventilation_temperature_setpoint);
}

void ventilation_save_settings(void)
{
    nvs_handle_t nvs_handle;
    esp_err_t err;
    
    err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error opening NVS namespace: %s", esp_err_to_name(err));
        return;
    }
    
    err = nvs_set_float(nvs_handle, "temp_setpoint", ventilation_temperature_setpoint);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error saving temp_setpoint: %s", esp_err_to_name(err));
    }
    
    err = nvs_set_str(nvs_handle, "vent_status", ventilation_status);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error saving vent_status: %s", esp_err_to_name(err));
    }
    
    err = nvs_set_i32(nvs_handle, "vent_percent", current_ventilation_percent);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error saving vent_percent: %s", esp_err_to_name(err));
    }
    
    err = nvs_set_u8(nvs_handle, "auto_control", ventilation_control_auto_state ? 1 : 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error saving auto_control: %s", esp_err_to_name(err));
    }
    
    err = nvs_set_u32(nvs_handle, "vent_coefficient", ventilation_open_coefficient);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error saving vent_coefficient: %s", esp_err_to_name(err));
    }
    
    err = nvs_set_i32(nvs_handle, "wind_speed", ventilation_wind_speed_setpoint);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error saving wind_speed: %s", esp_err_to_name(err));
    }
    
    err = nvs_set_float(nvs_handle, "vent_emerg_temp", ventilation_emergency_off_temperature);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error saving vent_emerg_temp: %s", esp_err_to_name(err));
    }
    
    err = nvs_commit(nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error committing NVS changes: %s", esp_err_to_name(err));
    }
    
    nvs_close(nvs_handle);
}

void ventilation_publish_status(void)
{
    cJSON *root = cJSON_CreateObject();
    cJSON_AddStringToObject(root, "status", ventilation_status);
    cJSON_AddNumberToObject(root, "percent", current_ventilation_percent);
    cJSON_AddBoolToObject(root, "auto_control", ventilation_control_auto_state);
    cJSON_AddStringToObject(root, "greenhouse_id", GREENHOUSE_ID);
    
    char *data = cJSON_Print(root);
    mqtt_publish(TOPIC_VENTILATION_STATUS, data);
    
    free(data);
    cJSON_Delete(root);
    
    ESP_LOGI(TAG, "Published ventilation status: %s, %d%%", ventilation_status, current_ventilation_percent);
}

void ventilation_publish_setpoints(void)
{
    cJSON *root = cJSON_CreateObject();
    cJSON_AddNumberToObject(root, "temperature", ventilation_temperature_setpoint);
    cJSON_AddNumberToObject(root, "coefficient", ventilation_open_coefficient);
    cJSON_AddNumberToObject(root, "wind_speed", ventilation_wind_speed_setpoint);
    cJSON_AddNumberToObject(root, "emergency_off_temperature", ventilation_emergency_off_temperature);
    cJSON_AddStringToObject(root, "greenhouse_id", GREENHOUSE_ID);
    
    char *data = cJSON_Print(root);
    mqtt_publish(TOPIC_VENTILATION_SETPOINTS_GET, data);
    
    free(data);
    cJSON_Delete(root);
} 