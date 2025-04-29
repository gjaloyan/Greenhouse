#ifndef WIFI_H
#define WIFI_H

/**
 * @brief Initialize WiFi in station mode and connect to AP
 */
void wifi_init_sta(void);

/**
 * @brief Reconnect to WiFi if disconnected
 */
void wifi_reconnect(void);

/**
 * @brief Check if WiFi is connected
 * 
 * @return true if connected, false otherwise
 */
bool wifi_is_connected(void);

#endif /* WIFI_H */ 