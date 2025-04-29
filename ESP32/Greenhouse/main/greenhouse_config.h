#ifndef GREENHOUSE_CONFIG_H
#define GREENHOUSE_CONFIG_H

// Pin definitions
#define SENSOR_RS485_RX_PIN   16   // UART2 RX 
#define SENSOR_RS485_TX_PIN   17   // UART2 TX
#define RELAY_RS485_RX_PIN    26   // UART1 RX
#define RELAY_RS485_TX_PIN    25   // UART1 TX

// UART configurations
#define UART_SENSOR  UART_NUM_2
#define UART_RELAY   UART_NUM_1
#define UART_BAUD_RATE 9600

// WiFi settings
#define WIFI_SSID       "TEAM-51"
#define WIFI_PASSWORD   "26839269."

// MQTT settings
#define MQTT_SERVER     "34.71.239.197"
#define MQTT_PORT       1883
#define MQTT_USER       "gjaloyan"
#define MQTT_PASSWORD   "26839269"
#define MQTT_CLIENT_ID  "ESP32_Greenhouse"
#define GREENHOUSE_ID   "Greenhouse1"

// MQTT topics
#define TOPIC_SHT20              "sensors/sht20"
#define TOPIC_SHT20_READ         "sensors/sht20/read"
#define TOPIC_BMP280             "sensors/bmp280"
#define TOPIC_BMP280_READ        "sensors/bmp280/read"
#define TOPIC_LDR                "sensors/ldr"
#define TOPIC_LDR_READ           "sensors/ldr/read"
#define TOPIC_RELAY_COMMAND_ON   "relay/command/on"
#define TOPIC_RELAY_COMMAND_OFF  "relay/command/off"
#define TOPIC_RELAY_STATUS       "relay/status"
#define TOPIC_RELAY_STATUS_GET   "relay/status/get"
#define TOPIC_GREENHOUSE_STATUS  "greenhouse/status"
#define TOPIC_GREENHOUSE_STATUS_GET "greenhouse/status/get"

// Ventilation topics
#define TOPIC_VENTILATION_COMMAND        "ventilation/command"
#define TOPIC_VENTILATION_STATUS         "ventilation/status"
#define TOPIC_VENTILATION_SETPOINTS      "ventilation/setpoints"
#define TOPIC_VENTILATION_SETPOINTS_GET  "ventilation/setpoints/get"
#define TOPIC_VENTILATION_COMMAND_MANUAL "ventilation/command/manual"
#define TOPIC_VENTILATION_COMMAND_AUTO   "ventilation/command/auto"

// Cooling topics
#define TOPIC_COOLING_COMMAND           "cooling/command"
#define TOPIC_COOLING_STATUS            "cooling/status"
#define TOPIC_COOLING_SETPOINTS         "cooling/setpoints"
#define TOPIC_COOLING_SETPOINTS_GET     "cooling/setpoints/get"
#define TOPIC_COOLING_COMMAND_VENTILATOR_START "cooling/command/ventilator/start"
#define TOPIC_COOLING_COMMAND_VENTILATOR_STOP  "cooling/command/ventilator/stop"

// NVS (Non-Volatile Storage) keys
#define NVS_NAMESPACE "greenhouse"

// Max values
#define MAX_RELAYS 32
#define MAX_COOLING_VENTILATORS 4

#endif /* GREENHOUSE_CONFIG_H */ 