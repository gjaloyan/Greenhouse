#ifndef SENSORS_H
#define SENSORS_H

#include <stdbool.h>

/**
 * @brief Initialize sensor hardware (UART, etc.)
 */
void init_sensors(void);

/**
 * @brief Read data from SHT20 temperature and humidity sensor
 * 
 * @return true if reading was successful, false otherwise
 */
bool read_sht20_data(void);

/**
 * @brief Read values from SHT20 sensor
 * 
 * @param temperature Pointer to store temperature value
 * @param humidity Pointer to store humidity value
 * @return true if reading was successful, false otherwise
 */
bool read_sht20_values(float *temperature, float *humidity);

/**
 * @brief Read wind speed from sensor
 * 
 * @return Current wind speed value
 */
int read_wind_speed(void);

/**
 * @brief Calculate Modbus CRC16 checksum
 * 
 * @param buf Buffer containing data
 * @param len Length of data
 * @return Calculated CRC16 value
 */
uint16_t modbus_crc16(uint8_t* buf, int len);

#endif /* SENSORS_H */ 