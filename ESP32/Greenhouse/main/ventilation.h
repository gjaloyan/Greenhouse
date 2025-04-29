#ifndef VENTILATION_H
#define VENTILATION_H

#include <stdbool.h>

/**
 * @brief Initialize ventilation system
 */
void ventilation_init(void);

/**
 * @brief Start opening ventilation
 */
void ventilation_control_open(void);

/**
 * @brief Start closing ventilation
 */
void ventilation_control_close(void);

/**
 * @brief Stop ventilation movement
 */
void ventilation_control_stop(void);

/**
 * @brief Open ventilation to a specific percentage
 * 
 * @param target_percent Target percentage (0-100)
 */
void ventilation_control_open_percent(int target_percent);

/**
 * @brief Check progress of ventilation movement
 * Stop ventilation when target is reached
 */
void ventilation_check_progress(void);

/**
 * @brief Automatic ventilation control based on temperature
 */
void ventilation_control_auto(void);

/**
 * @brief Set automatic ventilation control state
 * 
 * @param auto_state true for auto mode, false for manual mode
 */
void ventilation_set_auto_control(bool auto_state);

/**
 * @brief Set ventilation setpoints
 * 
 * @param temperature Temperature setpoint
 * @param coefficient Opening time coefficient
 * @param wind_speed Maximum allowed wind speed
 * @param emergency_off_temp Emergency shutdown temperature
 */
void ventilation_set_setpoints(float temperature, int coefficient, int wind_speed, float emergency_off_temp);

/**
 * @brief Load ventilation settings from non-volatile storage
 */
void ventilation_load_state(void);

/**
 * @brief Save ventilation settings to non-volatile storage
 */
void ventilation_save_settings(void);

/**
 * @brief Publish ventilation status via MQTT
 */
void ventilation_publish_status(void);

/**
 * @brief Publish ventilation setpoints via MQTT
 */
void ventilation_publish_setpoints(void);

#endif /* VENTILATION_H */ 