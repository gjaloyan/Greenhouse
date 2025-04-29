#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "cJSON.h"

#include "relays.h"
#include "sensors.h"  // For modbus_crc16
#include "mqtt.h"
#include "greenhouse_config.h"

static const char *TAG = "RELAYS";

// Array to store relay states
static bool relay_states[MAX_RELAYS] = {false};

// Forward declaration of helper function
static void send_relay_batch(int start_relay, int end_relay);
static char* esp_itoa(int value);

void init_relays(void)
{
    ESP_LOGI(TAG, "Initializing relays");
    
    // Configure UART for relays
    uart_config_t uart_config = {
        .baud_rate = UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };
    
    // Configure UART for relay communication
    ESP_ERROR_CHECK(uart_param_config(UART_RELAY, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_RELAY, RELAY_RS485_TX_PIN, RELAY_RS485_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(UART_RELAY, 256, 256, 0, NULL, 0));
    
    ESP_LOGI(TAG, "Relays UART initialized");
}

bool relay_turn_on(int relay_num)
{
    if (relay_num < 1 || relay_num > MAX_RELAYS) {
        ESP_LOGE(TAG, "Relay number %d out of range (1-%d)", relay_num, MAX_RELAYS);
        return false;
    }
    
    ESP_LOGI(TAG, "Turning ON relay %d", relay_num);
    
    // Prepare Modbus command to turn on relay
    uint8_t command[8];
    command[0] = 0x01;               // Device address
    command[1] = 0x05;               // Function 05 = Write Single Coil
    command[2] = 0x00;               // High byte of relay address
    command[3] = relay_num - 1;      // Low byte of relay address (0-31)
    command[4] = 0xFF;               // ON value (high byte)
    command[5] = 0x00;               // ON value (low byte)
    
    // Calculate CRC
    uint16_t crc = modbus_crc16(command, 6);
    command[6] = crc & 0xFF;         // Low byte of CRC
    command[7] = (crc >> 8) & 0xFF;  // High byte of CRC
    
    // Clear RX buffer
    uart_flush_input(UART_RELAY);
    
    // Send command
    uart_write_bytes(UART_RELAY, (const char*)command, 8);
    
    // Allow time for the relay module to process command
    vTaskDelay(10 / portTICK_PERIOD_MS);
    
    // Assume success for now
    relay_states[relay_num - 1] = true;
    
    // Save the updated state
    relays_save_state();
    
    // Verify relay status after a delay
    vTaskDelay(200 / portTICK_PERIOD_MS);
    
    bool actual_status = relay_get_status(relay_num);
    relay_states[relay_num - 1] = actual_status;
    
    if (actual_status) {
        ESP_LOGI(TAG, "Relay %d is verified to be ON", relay_num);
        return true;
    } else {
        ESP_LOGW(TAG, "WARNING: Relay %d should be ON but status check failed!", relay_num);
        return false;
    }
}

bool relay_turn_off(int relay_num)
{
    if (relay_num < 1 || relay_num > MAX_RELAYS) {
        ESP_LOGE(TAG, "Relay number %d out of range (1-%d)", relay_num, MAX_RELAYS);
        return false;
    }
    
    ESP_LOGI(TAG, "Turning OFF relay %d", relay_num);
    
    // Prepare Modbus command to turn off relay
    uint8_t command[8];
    command[0] = 0x01;               // Device address
    command[1] = 0x05;               // Function 05 = Write Single Coil
    command[2] = 0x00;               // High byte of relay address
    command[3] = relay_num - 1;      // Low byte of relay address (0-31)
    command[4] = 0x00;               // OFF value (high byte)
    command[5] = 0x00;               // OFF value (low byte)
    
    // Calculate CRC
    uint16_t crc = modbus_crc16(command, 6);
    command[6] = crc & 0xFF;         // Low byte of CRC
    command[7] = (crc >> 8) & 0xFF;  // High byte of CRC
    
    // Clear RX buffer
    uart_flush_input(UART_RELAY);
    
    // Send command
    uart_write_bytes(UART_RELAY, (const char*)command, 8);
    
    // Allow time for the relay module to process command
    vTaskDelay(10 / portTICK_PERIOD_MS);
    
    // Assume success for now
    relay_states[relay_num - 1] = false;
    
    // Save the updated state
    relays_save_state();
    
    // Verify relay status after a delay
    vTaskDelay(200 / portTICK_PERIOD_MS);
    
    bool actual_status = relay_get_status(relay_num);
    relay_states[relay_num - 1] = actual_status;
    
    if (!actual_status) {
        ESP_LOGI(TAG, "Relay %d is verified to be OFF", relay_num);
        return true;
    } else {
        ESP_LOGW(TAG, "WARNING: Relay %d should be OFF but status check failed!", relay_num);
        return false;
    }
}

bool relay_get_status(int relay_num)
{
    if (relay_num < 1 || relay_num > MAX_RELAYS) {
        ESP_LOGE(TAG, "Relay number %d out of range (1-%d)", relay_num, MAX_RELAYS);
        return false;
    }
    
    // Modbus RTU request to read all 32 relays
    uint8_t request[8] = {0x01, 0x01, 0x00, 0x00, 0x00, 0x20, 0x3D, 0xD2};
    
    // Clear RX buffer
    uart_flush_input(UART_RELAY);
    
    // Send request
    uart_write_bytes(UART_RELAY, (const char*)request, 8);
    
    // Wait for response (expect 9 bytes: 01 01 04 00 00 00 00 FB D1)
    uint8_t response[16];
    int len = 0;
    
    // Read response with timeout
    const int timeout_ms = 1000;
    int64_t start_time = esp_timer_get_time() / 1000;
    
    while ((esp_timer_get_time() / 1000 - start_time < timeout_ms) && len < 9) {
        int rx_bytes = uart_read_bytes(UART_RELAY, response + len, 9 - len, 20 / portTICK_PERIOD_MS);
        if (rx_bytes > 0) {
            len += rx_bytes;
        }
    }
    
    // Debug output
    ESP_LOGI(TAG, "Bytes received: %d", len);
    if (len) {
        char debug_buf[100] = {0};
        char temp[10];
        for (int i = 0; i < len; i++) {
            sprintf(temp, "%02X ", response[i]);
            strcat(debug_buf, temp);
        }
        ESP_LOGI(TAG, "Response: %s", debug_buf);
    }
    
    // Basic validation
    if (len < 9 || response[0] != 0x01 || response[1] != 0x01 || response[2] != 0x04) {
        ESP_LOGW(TAG, "Invalid/incomplete response");
        return relay_states[relay_num - 1]; // Return cached value
    }
    
    // CRC check
    uint16_t recvCRC = (response[8] << 8) | response[7];
    uint16_t calcCRC = modbus_crc16(response, 7);
    if (recvCRC != calcCRC) {
        ESP_LOGW(TAG, "CRC mismatch");
        return relay_states[relay_num - 1];
    }
    
    // Parse relay bit with reversed byte order: last data byte contains relays 1‑8
    uint8_t dataByteIndex = 3 - ((relay_num - 1) / 8);  // byte3 => relays 1‑8, byte0 => relays 25‑32
    uint8_t bitPos = (relay_num - 1) % 8;
    uint8_t dataByte = response[3 + dataByteIndex];
    
    bool state = (dataByte >> bitPos) & 0x01;
    
    ESP_LOGI(TAG, "Relay %d status: data byte index=%d value=0x%02X bitPos=%d => state=%d",
             relay_num, dataByteIndex, dataByte, bitPos, state);
    
    relay_states[relay_num - 1] = state;
    
    return state;
}

void relay_publish_status(int relay_num)
{
    if (relay_num < 1 || relay_num > MAX_RELAYS) {
        ESP_LOGE(TAG, "Relay number %d out of range (1-%d)", relay_num, MAX_RELAYS);
        return;
    }
    
    // Get current status
    bool status = relay_get_status(relay_num);
    
    // Create JSON message
    cJSON *root = cJSON_CreateObject();
    cJSON_AddStringToObject(root, "relay_id", esp_itoa(relay_num));
    cJSON_AddStringToObject(root, "state", status ? "ON" : "OFF");
    cJSON_AddStringToObject(root, "greenhouse_id", GREENHOUSE_ID);
    
    char *data = cJSON_Print(root);
    mqtt_publish(TOPIC_RELAY_STATUS, data);
    
    free(data);
    cJSON_Delete(root);
}

void relay_publish_all_status(void)
{
    // Send relays in batches of 8 to avoid exceeding message size limits
    send_relay_batch(1, 8);
    vTaskDelay(100 / portTICK_PERIOD_MS);
    send_relay_batch(9, 16);
    vTaskDelay(100 / portTICK_PERIOD_MS);
    send_relay_batch(17, 24);
    vTaskDelay(100 / portTICK_PERIOD_MS);
    send_relay_batch(25, 32);
}

// Helper function to send relays in smaller batches
static void send_relay_batch(int start_relay, int end_relay)
{
    cJSON *root = cJSON_CreateObject();
    
    for (int i = start_relay; i <= end_relay; i++) {
        char key[8];
        sprintf(key, "%d", i);
        cJSON_AddStringToObject(root, key, relay_states[i-1] ? "ON" : "OFF");
    }
    
    cJSON_AddStringToObject(root, "greenhouse_id", GREENHOUSE_ID);
    
    char *data = cJSON_Print(root);
    ESP_LOGI(TAG, "Publishing relay batch %d-%d", start_relay, end_relay);
    mqtt_publish(TOPIC_RELAY_STATUS, data);
    
    free(data);
    cJSON_Delete(root);
}

void relays_save_state(void)
{
    nvs_handle_t nvs_handle;
    esp_err_t err;
    
    err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error opening NVS namespace: %s", esp_err_to_name(err));
        return;
    }
    
    for (int i = 0; i < MAX_RELAYS; i++) {
        char key[16];
        sprintf(key, "relay%d", i + 1);
        err = nvs_set_u8(nvs_handle, key, relay_states[i] ? 1 : 0);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Error saving relay %d state: %s", i + 1, esp_err_to_name(err));
        }
    }
    
    err = nvs_commit(nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error committing NVS changes: %s", esp_err_to_name(err));
    }
    
    nvs_close(nvs_handle);
}

void relays_load_state(void)
{
    nvs_handle_t nvs_handle;
    esp_err_t err;
    
    err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error opening NVS namespace: %s", esp_err_to_name(err));
        return;
    }
    
    for (int i = 0; i < MAX_RELAYS; i++) {
        char key[16];
        sprintf(key, "relay%d", i + 1);
        
        uint8_t value;
        err = nvs_get_u8(nvs_handle, key, &value);
        
        if (err == ESP_OK) {
            relay_states[i] = (value == 1);
            
            // Apply saved state to hardware
            if (relay_states[i]) {
                relay_turn_on(i + 1);
            } else {
                relay_turn_off(i + 1);
            }
        } else if (err != ESP_ERR_NVS_NOT_FOUND) {
            ESP_LOGE(TAG, "Error loading relay %d state: %s", i + 1, esp_err_to_name(err));
        }
    }
    
    nvs_close(nvs_handle);
}

// Helper function for numeric to string conversion
static char* esp_itoa(int value)
{
    static char buffer[12]; // Large enough for int32
    sprintf(buffer, "%d", value);
    return buffer;
} 