#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "esp_system.h"
#include "esp_event.h"
#include "esp_log.h"
#include "mqtt_client.h"
#include "cJSON.h"

#include "mqtt.h"
#include "wifi.h"
#include "greenhouse_config.h"
#include "relays.h"
#include "sensors.h"
#include "ventilation.h"
#include "cooling.h"

static const char *TAG = "MQTT";

static esp_mqtt_client_handle_t mqtt_client = NULL;
static bool mqtt_connected = false;

// Forward declarations of callback functions
static void mqtt_handle_relay_commands(const char* topic, char* data);
static void mqtt_handle_ventilation_commands(const char* topic, char* data);
static void mqtt_handle_cooling_commands(const char* topic, char* data);

static esp_err_t mqtt_event_handler(esp_mqtt_event_handle_t event)
{
    switch (event->event_id) {
        case MQTT_EVENT_CONNECTED:
            ESP_LOGI(TAG, "MQTT Connected to broker");
            mqtt_connected = true;
            
            // Subscribe to topics
            esp_mqtt_client_subscribe(mqtt_client, TOPIC_SHT20_READ, 0);
            esp_mqtt_client_subscribe(mqtt_client, TOPIC_BMP280_READ, 0);
            esp_mqtt_client_subscribe(mqtt_client, TOPIC_LDR_READ, 0);
            esp_mqtt_client_subscribe(mqtt_client, TOPIC_RELAY_COMMAND_ON, 0);
            esp_mqtt_client_subscribe(mqtt_client, TOPIC_RELAY_COMMAND_OFF, 0);
            esp_mqtt_client_subscribe(mqtt_client, TOPIC_RELAY_STATUS_GET, 0);
            esp_mqtt_client_subscribe(mqtt_client, TOPIC_GREENHOUSE_STATUS_GET, 0);
            
            // Ventilation
            esp_mqtt_client_subscribe(mqtt_client, TOPIC_VENTILATION_COMMAND, 0);
            esp_mqtt_client_subscribe(mqtt_client, TOPIC_VENTILATION_STATUS, 0);
            esp_mqtt_client_subscribe(mqtt_client, TOPIC_VENTILATION_SETPOINTS, 0);
            esp_mqtt_client_subscribe(mqtt_client, TOPIC_VENTILATION_SETPOINTS_GET, 0);
            esp_mqtt_client_subscribe(mqtt_client, TOPIC_VENTILATION_COMMAND_MANUAL, 0);
            esp_mqtt_client_subscribe(mqtt_client, TOPIC_VENTILATION_COMMAND_AUTO, 0);
            
            // Cooling
            esp_mqtt_client_subscribe(mqtt_client, TOPIC_COOLING_COMMAND, 0);
            esp_mqtt_client_subscribe(mqtt_client, TOPIC_COOLING_STATUS, 0);
            esp_mqtt_client_subscribe(mqtt_client, TOPIC_COOLING_SETPOINTS, 0);
            esp_mqtt_client_subscribe(mqtt_client, TOPIC_COOLING_SETPOINTS_GET, 0);
            esp_mqtt_client_subscribe(mqtt_client, TOPIC_COOLING_COMMAND_VENTILATOR_START, 0);
            esp_mqtt_client_subscribe(mqtt_client, TOPIC_COOLING_COMMAND_VENTILATOR_STOP, 0);
            
            // Publish connected status
            publish_greenhouse_status();
            break;
            
        case MQTT_EVENT_DISCONNECTED:
            ESP_LOGI(TAG, "MQTT Disconnected from broker");
            mqtt_connected = false;
            break;
            
        case MQTT_EVENT_SUBSCRIBED:
            ESP_LOGI(TAG, "MQTT Subscribed, msg_id=%d", event->msg_id);
            break;
            
        case MQTT_EVENT_UNSUBSCRIBED:
            ESP_LOGI(TAG, "MQTT Unsubscribed, msg_id=%d", event->msg_id);
            break;
            
        case MQTT_EVENT_PUBLISHED:
            ESP_LOGI(TAG, "MQTT Published, msg_id=%d", event->msg_id);
            break;
            
        case MQTT_EVENT_DATA:
            ESP_LOGI(TAG, "MQTT Data:");
            ESP_LOGI(TAG, "Topic: %.*s", event->topic_len, event->topic);
            ESP_LOGI(TAG, "Data: %.*s", event->data_len, event->data);
            
            // Process the received message
            // Extract topic and data
            char topic[64] = {0};
            char data[512] = {0};
            
            // Copy topic and data to local buffers
            if (event->topic_len < sizeof(topic) - 1) {
                memcpy(topic, event->topic, event->topic_len);
                topic[event->topic_len] = '\0';
            } else {
                ESP_LOGE(TAG, "Topic too long, truncating");
                memcpy(topic, event->topic, sizeof(topic) - 1);
                topic[sizeof(topic) - 1] = '\0';
            }
            
            if (event->data_len < sizeof(data) - 1) {
                memcpy(data, event->data, event->data_len);
                data[event->data_len] = '\0';
            } else {
                ESP_LOGE(TAG, "Data too long, truncating");
                memcpy(data, event->data, sizeof(data) - 1);
                data[sizeof(data) - 1] = '\0';
            }
            
            // Check if message is for this greenhouse
            bool messageForThisGreenhouse = true;
            
            if (strstr(data, "{") != NULL) {
                // Message contains JSON
                cJSON *json = cJSON_Parse(data);
                
                if (json) {
                    cJSON *targetGreenhouse = cJSON_GetObjectItem(json, "target_greenhouse");
                    
                    if (targetGreenhouse && cJSON_IsString(targetGreenhouse)) {
                        if (strcmp(targetGreenhouse->valuestring, GREENHOUSE_ID) != 0) {
                            messageForThisGreenhouse = false;
                            ESP_LOGI(TAG, "Message is for a different greenhouse. Ignoring.");
                        }
                    }
                    
                    // Check if this is a message we sent ourselves
                    cJSON *greenhouseId = cJSON_GetObjectItem(json, "greenhouse_id");
                    if (greenhouseId && cJSON_IsString(greenhouseId)) {
                        if (strcmp(greenhouseId->valuestring, GREENHOUSE_ID) == 0) {
                            if (strcmp(topic, TOPIC_VENTILATION_STATUS) == 0) {
                                ESP_LOGI(TAG, "Ignoring our own ventilation status message");
                                messageForThisGreenhouse = false;
                            }
                        }
                    }
                    
                    cJSON_Delete(json);
                }
            }
            
            if (!messageForThisGreenhouse) {
                break;
            }
            
            // Route message to appropriate handler
            if (strstr(topic, "relay") != NULL) {
                mqtt_handle_relay_commands(topic, data);
            } else if (strstr(topic, "ventilation") != NULL) {
                mqtt_handle_ventilation_commands(topic, data);
            } else if (strstr(topic, "cooling") != NULL) {
                mqtt_handle_cooling_commands(topic, data);
            } else if (strcmp(topic, TOPIC_SHT20_READ) == 0) {
                // Publish sensor data if requested
                float temp, humidity;
                if (read_sht20_values(&temp, &humidity)) {
                    publish_sht20_data(temp, humidity);
                }
            } else if (strcmp(topic, TOPIC_GREENHOUSE_STATUS_GET) == 0) {
                publish_greenhouse_status();
            }
            break;
            
        case MQTT_EVENT_ERROR:
            ESP_LOGE(TAG, "MQTT Error");
            break;
            
        default:
            ESP_LOGI(TAG, "MQTT Other event id: %d", event->event_id);
            break;
    }
    return ESP_OK;
}

void mqtt_init(void)
{
    // Configure MQTT client
    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = "mqtt://" MQTT_SERVER,
        .broker.address.port = MQTT_PORT,
        .credentials.username = MQTT_USER,
        .credentials.authentication.password = MQTT_PASSWORD,
        .credentials.client_id = MQTT_CLIENT_ID,
    };
    
    mqtt_client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_register_event(mqtt_client, ESP_EVENT_ANY_ID, (esp_event_handler_t)mqtt_event_handler, NULL);
    esp_mqtt_client_start(mqtt_client);
    
    ESP_LOGI(TAG, "MQTT initialized, trying to connect to %s", MQTT_SERVER);
}

void mqtt_loop(void)
{
    // This function is called from the main loop
    // The actual MQTT client processing happens in the background
    // No need to do anything here as esp_mqtt_client handles events asynchronously
}

void mqtt_reconnect(void)
{
    if (!mqtt_is_connected() && wifi_is_connected()) {
        ESP_LOGI(TAG, "Reconnecting to MQTT broker");
        esp_mqtt_client_reconnect(mqtt_client);
        
        // Wait a bit for reconnection (non-blocking)
        vTaskDelay(500 / portTICK_PERIOD_MS);
    }
}

bool mqtt_is_connected(void)
{
    return mqtt_connected;
}

void mqtt_publish(const char* topic, const char* data)
{
    if (!mqtt_is_connected()) {
        ESP_LOGI(TAG, "Not connected to MQTT broker, trying to reconnect");
        mqtt_reconnect();
        if (!mqtt_is_connected()) {
            ESP_LOGE(TAG, "MQTT disconnected, can't publish");
            return;
        }
    }
    
    int msg_id = esp_mqtt_client_publish(mqtt_client, topic, data, 0, 1, 0);
    ESP_LOGI(TAG, "Published message to %s, msg_id=%d", topic, msg_id);
}

void publish_greenhouse_status(void)
{
    cJSON *root = cJSON_CreateObject();
    cJSON_AddStringToObject(root, "status", "online");
    cJSON_AddStringToObject(root, "client_id", MQTT_CLIENT_ID);
    cJSON_AddStringToObject(root, "greenhouse_id", GREENHOUSE_ID);
    
    char *data = cJSON_Print(root);
    mqtt_publish(TOPIC_GREENHOUSE_STATUS, data);
    
    free(data);
    cJSON_Delete(root);
}

void publish_sht20_data(float temperature, float humidity)
{
    cJSON *root = cJSON_CreateObject();
    cJSON_AddNumberToObject(root, "temperature", temperature);
    cJSON_AddNumberToObject(root, "humidity", humidity);
    cJSON_AddStringToObject(root, "client_id", MQTT_CLIENT_ID);
    cJSON_AddStringToObject(root, "greenhouse_id", GREENHOUSE_ID);
    
    char *data = cJSON_Print(root);
    mqtt_publish(TOPIC_SHT20, data);
    
    free(data);
    cJSON_Delete(root);
    
    ESP_LOGI(TAG, "SHT20 data sent: %.1f°C, %.1f%%", temperature, humidity);
}

// Handler for relay commands
static void mqtt_handle_relay_commands(const char* topic, char* data)
{
    int relayIndex = 0;
    const char* action = NULL;
    
    // Parse JSON data if present
    if (strstr(data, "{") != NULL) {
        cJSON *json = cJSON_Parse(data);
        if (json) {
            cJSON *actionJson = cJSON_GetObjectItem(json, "action");
            if (actionJson) {
                if (cJSON_IsNumber(actionJson)) {
                    relayIndex = actionJson->valueint;
                } else if (cJSON_IsString(actionJson)) {
                    action = actionJson->valuestring;
                }
            }
            cJSON_Delete(json);
        }
    } else {
        // Try to parse data as relay number directly
        relayIndex = atoi(data);
    }
    
    // Handle relay commands
    if (strcmp(topic, TOPIC_RELAY_COMMAND_ON) == 0) {
        if (relayIndex > 0 && relayIndex <= MAX_RELAYS) {
            relay_turn_on(relayIndex);
            relay_publish_status(relayIndex);
        }
    } else if (strcmp(topic, TOPIC_RELAY_COMMAND_OFF) == 0) {
        if (relayIndex > 0 && relayIndex <= MAX_RELAYS) {
            relay_turn_off(relayIndex);
            relay_publish_status(relayIndex);
        }
    } else if (strcmp(topic, TOPIC_RELAY_STATUS_GET) == 0) {
        if (action && strcmp(action, "all") == 0) {
            relay_publish_all_status();
        } else if (relayIndex > 0 && relayIndex <= MAX_RELAYS) {
            relay_publish_status(relayIndex);
        }
    }
}

// Handler for ventilation commands
static void mqtt_handle_ventilation_commands(const char* topic, char* data)
{
    // Parse JSON data
    if (strstr(data, "{") == NULL) {
        // Try to parse data as a percentage
        int percentage = atoi(data);
        if (percentage >= 0 && percentage <= 100) {
            if (strcmp(topic, TOPIC_VENTILATION_COMMAND) == 0) {
                ventilation_control_open_percent(percentage);
                ventilation_save_settings();
                return;
            }
        }
    }
    
    cJSON *json = cJSON_Parse(data);
    if (!json) {
        ESP_LOGE(TAG, "Failed to parse ventilation command JSON");
        return;
    }
    
    cJSON *actionJson = cJSON_GetObjectItem(json, "action");
    const char* action = NULL;
    int actionValue = 0;
    
    if (actionJson) {
        if (cJSON_IsNumber(actionJson)) {
            actionValue = actionJson->valueint;
        } else if (cJSON_IsString(actionJson)) {
            action = actionJson->valuestring;
        }
    }
    
    if (strcmp(topic, TOPIC_VENTILATION_COMMAND) == 0) {
        if (actionValue > 0) {
            ventilation_control_open_percent(actionValue);
            ventilation_save_settings();
        }
    } else if (strcmp(topic, TOPIC_VENTILATION_COMMAND) == 0 && action && strcmp(action, "auto") == 0) {
        ventilation_set_auto_control(true);
        ventilation_control_open_percent(0);
        ventilation_control_auto();
        ventilation_save_settings();
    } else if (strcmp(topic, TOPIC_VENTILATION_COMMAND) == 0 && action && strcmp(action, "manual") == 0) {
        ventilation_set_auto_control(false);
        ventilation_control_open_percent(0);
        ventilation_save_settings();
    } else if (strcmp(topic, TOPIC_VENTILATION_STATUS) == 0 && action && strcmp(action, "get") == 0) {
        ventilation_publish_status();
    } else if (strcmp(topic, TOPIC_VENTILATION_SETPOINTS) == 0) {
        float setpointTemp = 0;
        int setpointCoef = 0;
        int setpointWindSpeed = 0;
        float setpointEmergencyTemp = 0;
        
        cJSON *tempJson = cJSON_GetObjectItem(json, "ventilation_setpoint_temperature");
        if (tempJson && cJSON_IsNumber(tempJson)) {
            setpointTemp = (float)tempJson->valuedouble;
        }
        
        cJSON *coefJson = cJSON_GetObjectItem(json, "ventilation_setpoint_coefficient");
        if (coefJson && cJSON_IsNumber(coefJson)) {
            setpointCoef = coefJson->valueint;
        }
        
        cJSON *windJson = cJSON_GetObjectItem(json, "ventilation_setpoint_wind_speed");
        if (windJson && cJSON_IsNumber(windJson)) {
            setpointWindSpeed = windJson->valueint;
        }
        
        cJSON *emergencyJson = cJSON_GetObjectItem(json, "ventilation_emergency_off_temperature");
        if (emergencyJson && cJSON_IsNumber(emergencyJson)) {
            setpointEmergencyTemp = (float)emergencyJson->valuedouble;
        }
        
        ventilation_set_setpoints(setpointTemp, setpointCoef, setpointWindSpeed, setpointEmergencyTemp);
    } else if (strcmp(topic, TOPIC_VENTILATION_SETPOINTS_GET) == 0 && action && strcmp(action, "get") == 0) {
        ventilation_publish_setpoints();
    } else if (strcmp(topic, TOPIC_VENTILATION_COMMAND_MANUAL) == 0) {
        ventilation_set_auto_control(false);
        ventilation_save_settings();
    } else if (strcmp(topic, TOPIC_VENTILATION_COMMAND_AUTO) == 0) {
        ventilation_set_auto_control(true);
        ventilation_control_auto();
        ventilation_save_settings();
    }
    
    cJSON_Delete(json);
}

// Handler for cooling commands
static void mqtt_handle_cooling_commands(const char* topic, char* data)
{
    // Parse JSON data
    if (strstr(data, "{") == NULL) {
        // Try to parse data as a percentage or ventilator index
        int value = atoi(data);
        
        if (strcmp(topic, TOPIC_COOLING_COMMAND) == 0 && value >= 0 && value <= 100) {
            cooling_system_start(value);
            return;
        } else if (strcmp(topic, TOPIC_COOLING_COMMAND_VENTILATOR_START) == 0 && value >= 1 && value <= MAX_COOLING_VENTILATORS) {
            cooling_start_ventilator(value);
            return;
        } else if (strcmp(topic, TOPIC_COOLING_COMMAND_VENTILATOR_STOP) == 0 && value >= 1 && value <= MAX_COOLING_VENTILATORS) {
            cooling_stop_ventilator(value);
            return;
        }
    }
    
    cJSON *json = cJSON_Parse(data);
    if (!json) {
        ESP_LOGE(TAG, "Failed to parse cooling command JSON");
        return;
    }
    
    cJSON *actionJson = cJSON_GetObjectItem(json, "action");
    const char* action = NULL;
    
    if (actionJson && cJSON_IsString(actionJson)) {
        action = actionJson->valuestring;
    }
    
    if (strcmp(topic, TOPIC_COOLING_SETPOINTS_GET) == 0 && action && strcmp(action, "get") == 0) {
        cooling_publish_setpoints();
    } else if (strcmp(topic, TOPIC_COOLING_SETPOINTS) == 0) {
        float targetTemp = 0;
        float emergencyOffTemp = 0;
        bool autoControl = false;
        
        cJSON *targetTempJson = cJSON_GetObjectItem(json, "cooling_setpoint_temperature");
        if (targetTempJson && cJSON_IsNumber(targetTempJson)) {
            targetTemp = (float)targetTempJson->valuedouble;
        }
        
        cJSON *emergencyTempJson = cJSON_GetObjectItem(json, "cooling_emergency_off_temperature");
        if (emergencyTempJson && cJSON_IsNumber(emergencyTempJson)) {
            emergencyOffTemp = (float)emergencyTempJson->valuedouble;
        }
        
        cJSON *autoControlJson = cJSON_GetObjectItem(json, "cooling_control_auto_state");
        if (autoControlJson && cJSON_IsBool(autoControlJson)) {
            autoControl = autoControlJson->valueint ? true : false;
        }
        
        cooling_set_setpoints(targetTemp, emergencyOffTemp, autoControl);
    } else if (strcmp(topic, TOPIC_COOLING_STATUS) == 0 && action && strcmp(action, "get") == 0) {
        cooling_publish_status();
    } else if (strcmp(topic, TOPIC_COOLING_COMMAND) == 0 && action && strcmp(action, "manual") == 0) {
        cooling_set_auto_control(false);
        cooling_system_stop();
        cooling_save_settings();
    } else if (strcmp(topic, TOPIC_COOLING_COMMAND) == 0 && action && strcmp(action, "auto") == 0) {
        cooling_set_auto_control(true);
        cooling_control_auto();
        cooling_save_settings();
    }
    
    cJSON_Delete(json);
} 