#ifndef COOLING_H
#define COOLING_H

#include <stdbool.h>

/**
 * @brief Initialize cooling system
 */
void cooling_init(void);

/**
 * @brief Check if cooling system is active
 * 
 * @return true if cooling system is active, false otherwise
 */
bool cooling_is_active(void);

/**
 * @brief Start cooling system with specified intensity percentage
 * 
 * @param percent Cooling intensity (0-100%)
 */
void cooling_system_start(int percent);

/**
 * @brief Start water pump
 */
void cooling_system_start_water_pump(void);

/**
 * @brief Stop the entire cooling system
 */
void cooling_system_stop(void);

/**
 * @brief Stop cooling ventilators
 */
void cooling_system_stop_ventilators(void);

/**
 * @brief Stop cooling water pump
 */
void cooling_system_stop_water_pump(void);

/**
 * @brief Start a specific ventilator
 * 
 * @param ventilator_num Ventilator number (1-4)
 */
void cooling_start_ventilator(int ventilator_num);

/**
 * @brief Stop a specific ventilator
 * 
 * @param ventilator_num Ventilator number (1-4)
 */
void cooling_stop_ventilator(int ventilator_num);

/**
 * @brief Set cooling auto control state
 * 
 * @param auto_state true for auto mode, false for manual mode
 */
void cooling_set_auto_control(bool auto_state);

/**
 * @brief Automatic cooling control based on temperature
 */
void cooling_control_auto(void);

/**
 * @brief Set cooling system setpoints
 * 
 * @param target_temp Target temperature
 * @param emergency_off_temp Emergency shutdown temperature
 * @param auto_control Auto control state
 */
void cooling_set_setpoints(float target_temp, float emergency_off_temp, bool auto_control);

/**
 * @brief Load cooling system settings from non-volatile storage
 */
void cooling_load_state(void);

/**
 * @brief Save cooling system settings to non-volatile storage
 */
void cooling_save_settings(void);

/**
 * @brief Publish cooling system status via MQTT
 */
void cooling_publish_status(void);

/**
 * @brief Publish cooling system setpoints via MQTT
 */
void cooling_publish_setpoints(void);

#endif /* COOLING_H */ 