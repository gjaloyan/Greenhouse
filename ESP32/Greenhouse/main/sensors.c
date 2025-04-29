#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "driver/gpio.h"

#include "sensors.h"
#include "greenhouse_config.h"

static const char *TAG = "SENSORS";

// Global variables to store sensor readings
static float temperature = 0.0;
static float humidity = 0.0;
static int wind_speed = 0;

void init_sensors(void)
{
    ESP_LOGI(TAG, "Initializing sensors");
    
    // Configure UART for sensors
    uart_config_t uart_config = {
        .baud_rate = UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };
    
    // Configure UART for sensor communication
    ESP_ERROR_CHECK(uart_param_config(UART_SENSOR, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_SENSOR, SENSOR_RS485_TX_PIN, SENSOR_RS485_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(UART_SENSOR, 256, 256, 0, NULL, 0));
    
    ESP_LOGI(TAG, "Sensors UART initialized");
    
    // Read initial sensor values
    read_sht20_data();
}

uint16_t modbus_crc16(uint8_t* buf, int len)
{
    uint16_t crc = 0xFFFF;
    
    for (int pos = 0; pos < len; pos++) {
        crc ^= (uint16_t)buf[pos];  // XOR byte with current CRC
        
        for (int i = 8; i != 0; i--) {  // For each bit in byte
            if ((crc & 0x0001) != 0) {  // If LSB = 1
                crc >>= 1;              // Shift right by 1 bit
                crc ^= 0xA001;          // XOR with polynomial 0xA001
            } else {                    // Else if LSB = 0
                crc >>= 1;              // Just shift right
            }
        }
    }
    return crc;
}

bool read_sht20_data(void)
{
    bool success = read_sht20_values(&temperature, &humidity);
    if (success) {
        ESP_LOGI(TAG, "SHT20: Temperature=%.1f°C, Humidity=%.1f%%", temperature, humidity);
    } else {
        ESP_LOGE(TAG, "Failed to read SHT20 sensor");
    }
    return success;
}

bool read_sht20_values(float *temp, float *hum)
{
    const int maxAttempts = 10;
    int attempt = 0;
    
    while (attempt < maxAttempts) {
        // Build Modbus RTU request: Addr=1, Fn=0x04, Start=0x0001, Qty=0x0002
        uint8_t request[8] = {0x01, 0x04, 0x00, 0x01, 0x00, 0x02, 0x00, 0x00};
        uint16_t crc = modbus_crc16(request, 6);
        request[6] = crc & 0xFF;
        request[7] = crc >> 8;
        
        // Clear RX buffer
        uart_flush_input(UART_SENSOR);
        
        // Send request
        uart_write_bytes(UART_SENSOR, (const char*)request, 8);
        
        // Wait for response (expect 9 bytes: 01 04 04 T_hi T_lo H_hi H_lo CRC_lo CRC_hi)
        uint8_t resp[16];
        int len = 0;
        
        // Read response with timeout
        const int timeout_ms = 500;
        int64_t start_time = esp_timer_get_time() / 1000;
        
        while ((esp_timer_get_time() / 1000 - start_time < timeout_ms) && len < 9) {
            int rx_bytes = uart_read_bytes(UART_SENSOR, resp + len, 9 - len, 20 / portTICK_PERIOD_MS);
            if (rx_bytes > 0) {
                len += rx_bytes;
            }
        }
        
        if (len == 9 && resp[0] == 0x01 && resp[1] == 0x04 && resp[2] == 0x04) {
            uint16_t recvCRC = (resp[8] << 8) | resp[7];
            uint16_t calcCRC = modbus_crc16(resp, 7);
            
            if (recvCRC == calcCRC) {
                uint16_t rawT = (resp[3] << 8) | resp[4];
                uint16_t rawH = (resp[5] << 8) | resp[6];
                
                *temp = rawT / 10.0;
                *hum = rawH / 10.0;
                
                temperature = *temp;
                humidity = *hum;
                
                return true;
            }
        }
        
        ESP_LOGW(TAG, "SHT20 read failed, retry %d/%d", attempt + 1, maxAttempts);
        attempt++;
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
    
    ESP_LOGE(TAG, "Failed to read SHT20 after multiple attempts");
    return false;
}

int read_wind_speed(void)
{
    const int maxAttempts = 5;
    int attempt = 0;
    bool success = false;
    
    while (attempt < maxAttempts && !success) {
        // Build Modbus RTU request to read wind speed
        uint8_t request[8] = {0x01, 0x04, 0x00, 0x02, 0x00, 0x01, 0x00, 0x00};
        uint16_t crc = modbus_crc16(request, 6);
        request[6] = crc & 0xFF;
        request[7] = crc >> 8;
        
        // Clear RX buffer
        uart_flush_input(UART_SENSOR);
        
        // Send request
        uart_write_bytes(UART_SENSOR, (const char*)request, 8);
        
        // Wait for response
        uint8_t resp[16];
        int len = 0;
        
        // Read response with timeout
        const int timeout_ms = 500;
        int64_t start_time = esp_timer_get_time() / 1000;
        
        while ((esp_timer_get_time() / 1000 - start_time < timeout_ms) && len < 7) {
            int rx_bytes = uart_read_bytes(UART_SENSOR, resp + len, 7 - len, 20 / portTICK_PERIOD_MS);
            if (rx_bytes > 0) {
                len += rx_bytes;
            }
        }
        
        if (len == 7 && resp[0] == 0x01 && resp[1] == 0x04 && resp[2] == 0x02) {
            uint16_t recvCRC = (resp[6] << 8) | resp[5];
            uint16_t calcCRC = modbus_crc16(resp, 5);
            
            if (recvCRC == calcCRC) {
                uint16_t rawWindSpeed = (resp[3] << 8) | resp[4];
                
                // Convert to actual wind speed (assuming 0.1 m/s resolution)
                wind_speed = rawWindSpeed / 10;
                success = true;
            }
        }
        
        if (!success) {
            ESP_LOGW(TAG, "Wind speed read failed, retry %d/%d", attempt + 1, maxAttempts);
            attempt++;
            vTaskDelay(100 / portTICK_PERIOD_MS);
        }
    }
    
    if (!success) {
        ESP_LOGE(TAG, "Failed to read wind speed after multiple attempts");
        // Return cached value or 0 if no valid reading
    }
    
    return wind_speed;
} 