#ifndef MQTT_H
#define MQTT_H

#include <stdbool.h>

/**
 * @brief Initialize MQTT client and settings
 */
void mqtt_init(void);

/**
 * @brief Process MQTT messages and maintain connection
 */
void mqtt_loop(void);

/**
 * @brief Connect or reconnect to MQTT broker
 */
void mqtt_reconnect(void);

/**
 * @brief Check if MQTT client is connected
 * 
 * @return true if connected, false otherwise
 */
bool mqtt_is_connected(void);

/**
 * @brief Send data to an MQTT topic
 * 
 * @param topic The topic to publish to
 * @param data The data to publish
 */
void mqtt_publish(const char* topic, const char* data);

/**
 * @brief Publish greenhouse status (online)
 */
void publish_greenhouse_status(void);

/**
 * @brief Publish sensor data
 */
void publish_sht20_data(float temperature, float humidity);

#endif /* MQTT_H */ 