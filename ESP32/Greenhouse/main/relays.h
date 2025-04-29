#ifndef RELAYS_H
#define RELAYS_H

#include <stdbool.h>

/**
 * @brief Initialize relays hardware (UART, etc.)
 */
void init_relays(void);

/**
 * @brief Turn on a specific relay
 * 
 * @param relay_num Relay number (1-32)
 * @return true if successful, false otherwise
 */
bool relay_turn_on(int relay_num);

/**
 * @brief Turn off a specific relay
 * 
 * @param relay_num Relay number (1-32)
 * @return true if successful, false otherwise
 */
bool relay_turn_off(int relay_num);

/**
 * @brief Get status of a specific relay
 * 
 * @param relay_num Relay number (1-32)
 * @return true if relay is on, false if off or error
 */
bool relay_get_status(int relay_num);

/**
 * @brief Publish status of a specific relay via MQTT
 * 
 * @param relay_num Relay number (1-32)
 */
void relay_publish_status(int relay_num);

/**
 * @brief Publish status of all relays via MQTT
 */
void relay_publish_all_status(void);

/**
 * @brief Save relay states to non-volatile storage
 */
void relays_save_state(void);

/**
 * @brief Load relay states from non-volatile storage
 */
void relays_load_state(void);

#endif /* RELAYS_H */ 