idf_component_register(SRCS "main.c" "wifi.c" "mqtt.c" "sensors.c" "relays.c" "ventilation.c" "cooling.c"
                    INCLUDE_DIRS "."
                    REQUIRES nvs_flash esp_wifi mqtt esp_event driver cJSON)
