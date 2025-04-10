#include <ModbusMaster.h>
#include <WiFi.h>          // Changed from ESP8266WiFi.h
#include <PubSubClient.h>
#include <ArduinoJson.h>
// #include <Hash.h>
#include <Adafruit_Sensor.h>
#include <Preferences.h>
#include <esp_wifi.h>      // For esp_wifi_set_ps function

Preferences preferences;

// ESP32 pin definitions
#define RS485_TX_PIN   17   // UART2 TX
#define RS485_RX_PIN   16   // UART2 RX
#define RS485_CONTROL_PIN 4 // DE/RE pin for RS485

// Use Hardware Serial for relay control
#define RELAY_TX_PIN   14   // UART1 TX
#define RELAY_RX_PIN   15   // UART1 RX

// Define HardwareSerial for relay control (ESP32 has multiple hardware serial ports)
HardwareSerial relaySerial(1); // Use UART1 for relay control
HardwareSerial rs485Serial(2); // Use UART2 for RS485/Modbus


// Настройки WiFi
const char* ssid =   "TEAM-51";
const char* password = "26839269.";

// Настройки MQTT
const char* mqtt_server = "34.71.239.197";
const int mqtt_port = 1883;
const char* mqtt_user = "gjaloyan";
const char* mqtt_password = "26839269";
const char* mqtt_client_id = "ESP32_Greenhouse";
const char* greenhouse_id = "Greenhouse1"; // Уникальный ID этой теплицы



// MQTT Topics
const char* topic_sht20 = "sensors/sht20";
const char* topic_sht20_read = "sensors/sht20/read";
const char* topic_bmp280 = "sensors/bmp280";
const char* topic_bmp280_read = "sensors/bmp280/read";
const char* topic_ldr = "sensors/ldr";
const char* topic_ldr_read = "sensors/ldr/read";
const char* topic_relay_command = "relay/command";
const char* topic_relay_status = "relay/status";
const char* topic_relay_status_get = "relay/status/get";
const char* topic_greenhouse_status = "greenhouse/status";
const char* topic_greenhouse_status_get = "greenhouse/status/get";

//Ventilation Topics
const char* topic_ventilation_command = "ventilation/command";
const char* topic_ventilation_status = "ventilation/status";
const char* ventilation_setpoints = "ventilation/setpoints";
const char* ventilation_setpoints_get = "ventilation/setpoints/get";
// const char* ventilation_command_manual = "ventilation/command/manual";
// const char* ventilation_command_auto = "ventilation/command/auto";

//Cooling Topics 
const char* topic_cooling_command = "cooling/command";
const char* topic_cooling_status = "cooling/status";
const char* topic_cooling_setpoints = "cooling/setpoints";
const char* topic_cooling_setpoints_get = "cooling/setpoints/get";
const char* topic_cooling_command_ventilator_start = "cooling/command/ventilator/start";
const char* topic_cooling_command_ventilator_stop = "cooling/command/ventilator/stop";
// const char* topic_cooling_command_manual = "cooling/command/manual";
// const char* topic_cooling_command_auto = "cooling/command/auto";



StaticJsonDocument<200> RelayState;


WiFiClient espClient;
PubSubClient mqttClient(espClient);
ModbusMaster node;

// Variables
unsigned long lastMsg = 0;
unsigned long sensorPollInterval = 10000; // 3 seconds
bool relayStatus[4] = {false, false, false, false};
float temperature = 0.0;
float humidity    = 0.0;

// Ventilation System Variables
const char* ventilationStatus = "closed";
float ventilation_temperature_setpoint = 26.0;
bool ventilation_control_auto_state = false;
unsigned long ventilation_open_coefficient = 500;

unsigned long ventilation_start_time = 0;
unsigned long ventilation_open_duration = 0;
int ventilation_percent_target = 0;
bool ventilation_in_progress = false;
int current_ventilation_percent = 0;
int ventilation_wind_speed_setpoint = 0;
int wind_speed_current = 0;
float ventilation_emergency_off_temperature = 0;

//Cooling System Variables
float cooling_target_temperature = 0;
float cooling_emergency_off_temperature = 0;
bool cooling_control_auto_state = false;
bool cooling_system_active = false;
bool cooling_water_pump_status = false;
bool cooling_ventilators_status[4] = {false, false, false, false};
// int cooling_ventilators_intensity_setpoint = 0;

void setup() {
  // Configure RS485 control pin
  pinMode(RS485_CONTROL_PIN, OUTPUT);
  digitalWrite(RS485_CONTROL_PIN, LOW); // Start in receive mode

  // Start Serial for debugging
  Serial.begin(9600);
  Serial.println("\n\nInitializing system...");
  
  // Initialize relay serial
  relaySerial.begin(115200, SERIAL_8N1, RELAY_RX_PIN, RELAY_TX_PIN);
  
  // Initialize RS485 serial
  rs485Serial.begin(9600, SERIAL_8N1, RS485_RX_PIN, RS485_TX_PIN);
  
  setup_wifi();
  mqttClient.setServer(mqtt_server, mqtt_port);
  mqttClient.setCallback(mqtt_callback);
  
  // Initialize Modbus with RS485 serial
  node.begin(1, rs485Serial);
  node.preTransmission(preTransmission);
  delay(20);
  node.postTransmission(postTransmission);

    // Open preferences with namespace "greenhouse"
  preferences.begin("greenhouse", false);
  
  // Restore saved values
  // Ventilation
  ventilation_temperature_setpoint = preferences.getFloat("temp_setpoint", 20.0); // Default 26.0
  current_ventilation_percent = preferences.getInt("vent_percent", 0); // Default 0%
  ventilation_control_auto_state = preferences.getBool("auto_control", false); // Default false
  ventilation_wind_speed_setpoint = preferences.getInt("wind_speed", 0); // Default 0%
  ventilation_emergency_off_temperature = preferences.getFloat("ventilation_emergency_off_temperature", 0); // Default 0%
  // Cooling
  cooling_target_temperature = preferences.getFloat("cooling_target_temperature", 0); // Default 0%
  cooling_emergency_off_temperature = preferences.getFloat("cooling_emergency_off_temperature", 0); // Default 0%
  cooling_control_auto_state = preferences.getBool("cooling_control_auto_state", false); // Default false
  cooling_system_active = preferences.getBool("cooling_system_active", false); // Default false
  
  // You could also load relay states
  relayStatus[0] = preferences.getBool("relay1", false);
  relayStatus[1] = preferences.getBool("relay2", false);
  relayStatus[2] = preferences.getBool("relay3", false);
  relayStatus[3] = preferences.getBool("relay4", false);

  // Ventilators
  cooling_ventilators_status[0] = preferences.getBool("v1", false);
  cooling_ventilators_status[1] = preferences.getBool("v2", false);
  cooling_ventilators_status[2] = preferences.getBool("v3", false);
  cooling_ventilators_status[3] = preferences.getBool("v4", false);

    // Apply loaded states to hardware
  for (int i = 0; i < 4; i++) {
    if (relayStatus[i]) {
      turnOnRelay(i+1);
    } else {
      turnOffRelay(i+1);
    }
  }

  for (int i = 0; i < 4; i++) {
    if (cooling_ventilators_status[i]) {
      startVentilator(i+1);
    } else {
      stopVentilator(i+1);
    }
  }

}

  

void loop() {

  if (!mqttClient.connected()) {
    reconnectMQTT();
  }
  mqttClient.loop();

if (ventilation_in_progress && (millis() - ventilation_start_time >= ventilation_open_duration)) {
  ventilation_control_stop();
  ventilation_in_progress = false;
  Serial.println("Ventilation movement complete. Now at " + String(current_ventilation_percent) + "%");
}

if (wind_speed_current > ventilation_wind_speed_setpoint){
  ventilation_control_stop();
  ventilation_in_progress = false;
  Serial.println("Vindspeed is very high. Ventilation stopped");
}



  // Publish sensor data periodically
  unsigned long now = millis();
  if (now - lastMsg > sensorPollInterval) {
    lastMsg = now;
    readSHT20Data();
    ventilation_control_auto();
    // readWindSpeed();
  }
}

// Function to initialize and connect to WiFi
void setup_wifi() {
  delay(10);
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  // Set WiFi to station mode 
  WiFi.mode(WIFI_STA);
  
  // Disable WiFi power saving mode
  esp_wifi_set_ps(WIFI_PS_NONE);
  
  // Begin connection
  WiFi.begin(ssid, password);

  // Wait for connection with timeout
  int timeout_counter = 0;
  while (WiFi.status() != WL_CONNECTED && timeout_counter < 50) {
    delay(500);
    Serial.print(".");
    timeout_counter++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("");
    Serial.println("WiFi connected");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("");
    Serial.println("WiFi connection FAILED - continuing without WiFi");
  }
}


void reconnectMQTT() {
  // Повторяем попытки подключения к MQTT-серверу до успеха
  while (!mqttClient.connected()) {
    String clientId = mqtt_client_id;
    Serial.print("Attempting MQTT connection...");
    // Пытаемся подключиться с уникальным идентификатором клиента
    if (mqttClient.connect(clientId.c_str(), mqtt_user, mqtt_password)) {
      Serial.println("connected");

      // Subscribe to topics
      mqttClient.subscribe(topic_sht20_read);
      mqttClient.subscribe(topic_bmp280_read);
      mqttClient.subscribe(topic_ldr_read);
      mqttClient.subscribe(topic_relay_command);
      mqttClient.subscribe(topic_relay_status_get);
      mqttClient.subscribe(topic_ventilation_command);
      mqttClient.subscribe(topic_ventilation_status); 
      mqttClient.subscribe(topic_greenhouse_status_get);
      mqttClient.subscribe(ventilation_setpoints);
      mqttClient.subscribe(ventilation_setpoints_get);

    } else {
      Serial.print("failed, rc=");
      Serial.print(mqttClient.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }
}

void publishGreenhouseStatus(){
  StaticJsonDocument<200> doc;
  doc["status"] = "online";
  doc["client_id"] = mqtt_client_id;
  doc["greenhouse_id"] = greenhouse_id;
  
  char buffer[200];
  serializeJson(doc, buffer);
  
  sendMQTTData(topic_greenhouse_status, buffer);
}



void mqtt_callback(char* topic, byte* payload, unsigned int length) {
  // Convert payload to string
  char message[length + 1];
  for (int i = 0; i < length; i++) {
    message[i] = (char)payload[i];
  }
  message[length] = '\0';
  
  String topicStr = String(topic);
  String messageStr = String(message);
  const char* action = ""; 
  int actionValue = 0; // Добавляем переменную для числовых значений действий
  int actionValue2 = 0; // Добавляем переменную для числовых значений действий
  float setpointTemperature = 0;
  int setpointCoefficient = 0;
  int setpointWindSpeed = 0;

  float coolingSetpointTemperature = 0;
  float coolingEmergencyOffTemperature = 0;
  bool coolingControlAutoState = false;
  
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("]: ");
  Serial.println(messageStr);

  // Проверяем, адресовано ли сообщение этой теплице
  // Если сообщение содержит JSON, проверяем поле target_greenhouse
  bool messageForThisGreenhouse = true; // По умолчанию обрабатываем
  
  if (messageStr.indexOf("{") >= 0) {
    // Если сообщение содержит JSON структуру
    StaticJsonDocument<400> doc;
    DeserializationError error = deserializeJson(doc, messageStr);

    if (!error && doc.containsKey("target_greenhouse")) {
      // Если сообщение содержит поле target_greenhouse
      const char* targetId = doc["target_greenhouse"];
      
      // Проверяем, есть ли поле action
      if (doc.containsKey("action")) {
        // Сохраняем значение action в зависимости от его типа
        if (doc["action"].is<int>()) {
          // Если action - число (для вентиляции)
          actionValue = doc["action"].as<int>();
          action = ""; // Строковое значение пустое, так как action числовой
        } else if (doc["action"].is<const char*>()) {
          // Если action - строка (для других команд)
          action = doc["action"].as<const char*>();
        }
      }


      if (doc.containsKey("ventilation_setpoint_temperature")) {
        setpointTemperature = doc["ventilation_setpoint_temperature"].as<float>();
        setpointCoefficient = doc["ventilation_setpoint_coefficient"].as<int>();
        setpointWindSpeed = doc["ventilation_setpoint_wind_speed"].as<int>();
        ventilation_emergency_off_temperature = doc["ventilation_emergency_off_temperature"].as<float>();
      }
      if (doc.containsKey("cooling_setpoint_temperature")) {
        coolingSetpointTemperature = doc["cooling_setpoint_temperature"].as<float>();
        coolingEmergencyOffTemperature = doc["cooling_emergency_off_temperature"].as<float>();
        coolingControlAutoState = doc["cooling_control_auto_state"].as<bool>();
      }

      messageForThisGreenhouse = (strcmp(targetId, greenhouse_id) == 0);
      
      if (!messageForThisGreenhouse) {
        Serial.println("Message is for a different greenhouse. Ignoring.");
        return; // Не обрабатываем сообщение
      }
    }
    
    // Проверка: если сообщение содержит наш собственный greenhouse_id и это сообщение о статусе вентиляции,
    // то это наше собственное сообщение, которое мы отправили - игнорируем его
    if (!error && doc.containsKey("greenhouse_id") && strcmp(doc["greenhouse_id"], greenhouse_id) == 0) {
      if (topicStr == topic_ventilation_status) {
        Serial.println("Ignoring our own ventilation status message");
        return; // Пропускаем обработку нашего собственного сообщения
      }
    }
  }
  
  String actionStr = String(action);
 
  // Продолжаем обработку сообщения, если оно для этой теплицы
  // Handle sensor read requests
  if (topicStr == topic_sht20_read && actionStr == "request") {
    publishSHT20Data();
  } 
  else if (topicStr == topic_bmp280_read && actionStr == "request") {
    // publishBMP280Data();
  }
  else if (topicStr == topic_ldr_read && actionStr == "request") {
    // publishLDRData();
  }
    // Handle relay commands
  else if (topicStr == topic_relay_command) {
    handleRelayCommand(actionStr);
  }

  // Ventilation
  else if (topicStr == topic_ventilation_command) {
    ventilation_control_open_percent(actionValue);
    saveVentilationSettings();
  }
  else if (topicStr == topic_ventilation_command && actionStr == "auto") {
    ventilation_control_auto_state = true;
    ventilation_control_open_percent(0);
    ventilation_control_auto();
    saveVentilationSettings();
  }
  else if (topicStr == topic_ventilation_command && actionStr == "manual") {
    ventilation_control_auto_state = false;
    ventilation_control_open_percent(0);
    saveVentilationSettings();
  }
  else if (topicStr == topic_ventilation_status && actionStr == "get") {
    publishVentilationStatus();
  }
  else if (topicStr == topic_greenhouse_status_get) {
    publishGreenhouseStatus();
  }
  else if (topicStr == ventilation_setpoints) {
    setVentilationSetpoints(setpointTemperature, setpointCoefficient, setpointWindSpeed, ventilation_emergency_off_temperature);
  }
  else if (topicStr == ventilation_setpoints_get && actionStr == "get") {
    publishVentilationSetpoints();
  }

  // Cooling
  else if (topicStr == topic_cooling_setpoints_get && actionStr == "get") {
    publishCoolingSetpoints();
  }
  else if (topicStr == topic_cooling_setpoints) {
    setCoolingSetpoints(coolingSetpointTemperature, coolingEmergencyOffTemperature, coolingControlAutoState);
  }
  else if (topicStr == topic_cooling_status && actionStr == "get") {
    publishCoolingStatus();
  }
  else if (topicStr == topic_cooling_command) {
    cooling_system_start(actionValue);
  }
  else if (topicStr == topic_cooling_command_ventilator_start) {
    startVentilator(actionValue);
  }
  else if (topicStr == topic_cooling_command_ventilator_stop) {
    stopVentilator(actionValue);
  }
  else if (topicStr == topic_cooling_command && actionStr == "manual") {
    cooling_control_auto_state = false;
    cooling_system_stop();
    saveCoolingSettings();
  }
  else if (topicStr == topic_cooling_command && actionStr == "auto") {
    cooling_control_auto_state = true;
    cooling_control_auto();
    saveCoolingSettings();
  }

  // Handle status requests
  else if (topicStr == topic_relay_status_get) {
    if (actionStr == "all") {
      publishAllRelayStatus();
    } else {
      // Check if it's a specific relay request
      if (actionStr == "r1") publishRelayStatus(1);
      else if (actionStr == "r2") publishRelayStatus(2);
      else if (actionStr == "r3") publishRelayStatus(3);
      else if (actionStr == "r4") publishRelayStatus(4);
    }
  }
}

// Handle relay command
void handleRelayCommand(String command) {
  int relayIndex = 1;
  bool turnOn = true;
  
  // Extract relay index (1-based in command, 0-based in array)
  if (command.startsWith("r1")) {
    relayIndex = 1;
  } else if (command.startsWith("r2")) {
    relayIndex = 2;
  } else if (command.startsWith("r3")) {
    relayIndex = 3;
  } else if (command.startsWith("r4")) {
    relayIndex = 4;
  } else if (command.startsWith("v1")) {
    relayIndex = 4;
  }
  
  
  // Check if this is an OFF command
  if (command.endsWith("_OFF")) {
    turnOn = false;
  }
  
  // Apply the command if valid
  if (relayIndex >= 0) {
    // Get physical pin
    // int pin;
    // switch (relayIndex) {
    //   case 0: pin = RELAY1_PIN; break;
    //   case 1: pin = RELAY2_PIN; break;
    //   case 2: pin = RELAY3_PIN; break;
    //   case 3: pin = RELAY4_PIN; break;
    // }
    
    // Set pin state
    if(turnOn){
      turnOnRelay(relayIndex);
    }
    else{
      turnOffRelay(relayIndex);
    }
    relayStatus[relayIndex -1] = turnOn;
    
    // Update status
    publishRelayStatus(relayIndex);
  }
}

// Publish data from all sensors
void publishSensorData() {
  publishSHT20Data();
  // publishBMP280Data();
  // publishLDRData();
}

void sendMQTTData(const char* topic, const char* data) {
    if (!mqttClient.connected()) {
    reconnectMQTT();
  }
  mqttClient.publish(topic, data);
};

void sendMQTTMessage(const char* topic, const char* message) {
  if (!mqttClient.connected()) {
    reconnectMQTT();
  }
  mqttClient.publish(topic, message, true);
}


void turnOnRelay(int relayNum) {
  byte command[] = {0xA0, (byte)relayNum, 0x01, (byte)(0xA0 + relayNum + 1)};
  int bytesWritten = relaySerial.write(command, sizeof(command));
  delay(1000);
  if (bytesWritten == sizeof(command)) {
    int bytesWritten = relaySerial.write(command, sizeof(command));
    Serial.println("Relay " + String(relayNum) + " turned on successfully");
    relayStatus[relayNum - 1] = true;
    saveRelayState(relayNum, true);
  } else {
    Serial.println("Error writing to relay serial port");
    relayStatus[relayNum - 1] = false;
  }
}

void turnOffRelay(int relayNum) {
  byte command[] = {0xA0, (byte)relayNum, 0x00, (byte)(0xA0 + relayNum)};
  int bytesWritten = relaySerial.write(command, sizeof(command));
  delay(1000);
  if (bytesWritten == sizeof(command)) {
    int bytesWritten = relaySerial.write(command, sizeof(command));
    Serial.println("Relay " + String(relayNum) + " turned Off successfully");
    relayStatus[relayNum - 1] = false;
    saveRelayState(relayNum, false);
  } else {
    Serial.println("Error writing to relay serial port");
    relayStatus[relayNum - 1] = false;
  }
}


// Function to save relay state
void saveRelayState(int relayNum, bool state) {
  String key = "relay" + String(relayNum);
  preferences.putBool(key.c_str(), state);
}


// Вызывается перед отправкой запроса (TX-режим)
void preTransmission() {
  digitalWrite(RS485_CONTROL_PIN, HIGH);
}
// Вызывается после отправки запроса (RX-режим)
void postTransmission() {
  digitalWrite(RS485_CONTROL_PIN, LOW);
}

void readWindSpeed(){
  const int maxAttempts = 50;  // Maximum number of retry attempts
  int attempt = 0;
  bool success = false;
  while (attempt < maxAttempts && !success) {
    // Установить ID устройства (slave ID) для конкретного датчика
    // node.setServerID(2);  // Поменяйте на ID вашего датчика (обычно 1-247)
    // Чтение данных с конкретного регистра
    uint8_t result = node.readInputRegisters(2, 1);  // Пример: адрес 2, 1 регистра
    if (result == node.ku8MBSuccess) {
      uint16_t rawWindSpeed = node.getResponseBuffer(0);

      // // Для float значения (4 байта / 2 регистра)
      // union {
      //   uint16_t words[2];
      //   float value;
      // } data;

      // data.words[0] = rawWindSpeed;
      // data.words[1] = 0;
      // float sensorValue = data.value;


      wind_speed_current = rawWindSpeed / 10.0;

      success = true;

    } else {
      Serial.print("Modbus wind speed read error. Code: 0x");
      Serial.println(result, HEX);
      attempt++;
      // Increase delay to allow the sensor more time to respond
      delay(500);  
    }
  }

  if (!success) {
    Serial.println("Failed to read wind speed sensor data after multiple attempts.");
  }
  // Read wind speed from wind speed sensor
  // TODO: Implement wind speed reading
  wind_speed_current = 0; // Placeholder value
}


void readSHT20Data() {
  const int maxAttempts = 50;  // Maximum number of retry attempts
  int attempt = 0;
  bool success = false;
  while (attempt < maxAttempts && !success) {
    // Set RS485 to TX mode before reading
    // node.preTransmission(preTransmission);
    uint8_t result = node.readInputRegisters(1, 2);
    // uint8_t result = node.readHoldingRegisters(1, 2);  // Try this instead
    // Always set RS485 back to RX mode after the read attempt
    // node.postTransmission(postTransmission);
    if (result == node.ku8MBSuccess) {
      uint16_t rawTemperature = node.getResponseBuffer(0);
      uint16_t rawHumidity    = node.getResponseBuffer(1);

      temperature = rawTemperature / 10.0;
      humidity    = rawHumidity / 10.0;

      success = true;

    } else {
      Serial.print("Modbus temperature and humidity read error. Code: 0x");
      Serial.println(result, HEX);
      attempt++;
      // Increase delay to allow the sensor more time to respond
      delay(500);  
    }
  }

  if (!success) {
    Serial.println("Failed to read temperature and humidity sensor data after multiple attempts.");
  }
}

void publishSHT20Data(){
  readSHT20Data();
  StaticJsonDocument<200> doc;
  doc["temperature"] = temperature;
  doc["humidity"] = humidity;
  doc["client_id"] = mqtt_client_id;
  doc["greenhouse_id"] = greenhouse_id;

  char jsonBuffer[200];
  serializeJson(doc, jsonBuffer);
  
  sendMQTTData(topic_sht20, jsonBuffer);
  Serial.println("Data Sent");
}


// Publish status of all relays
void publishAllRelayStatus() {
  StaticJsonDocument<200> doc;
  doc["r1"] = relayStatus[0] ? "ON" : "OFF";
  doc["r2"] = relayStatus[1] ? "ON" : "OFF";
  doc["r3"] = relayStatus[2] ? "ON" : "OFF";
  doc["r4"] = relayStatus[3] ? "ON" : "OFF";
  doc["greenhouse_id"] = greenhouse_id;
  
  char buffer[200];
  serializeJson(doc, buffer);
  
  // Publish
  sendMQTTData(topic_relay_status, buffer);
}

// Publish status of a specific relay
void publishRelayStatus(int relayIndex) {
  if (relayIndex < 0 || relayIndex > 4) return;
  
  String relayId = "r" + String(relayIndex);
  int relay = relayIndex - 1;
  String state;
  if(relayStatus[relay]){
    state = "ON";
  }else{
    state = "OFF";
  }

  
  StaticJsonDocument<100> doc;
  doc[relayId] = state;
  doc["greenhouse_id"] = greenhouse_id;
  
  char buffer[100];
  serializeJson(doc, buffer);
  
  // Publish
  sendMQTTData(topic_relay_status, buffer);
} 


//Ventilation System
void ventilation_control_open(){
  turnOnRelay(4);
  if(relayStatus[3] == true){
    ventilationStatus = "open";
    current_ventilation_percent = 100;
  }else{
    ventilationStatus = "open_error";
  }
}

void ventilation_control_close(){
  turnOnRelay(3);
  if(relayStatus[2] == true){
    ventilationStatus = "closed";
    current_ventilation_percent = 0;
  }else{
    ventilationStatus = "closed_error";
  }
}

void ventilation_control_stop(){
  turnOffRelay(3);
  turnOffRelay(4);
  if(relayStatus[2] == false && relayStatus[3] == false){
    ventilationStatus = "stopped";
    current_ventilation_percent = ventilation_percent_target;
  }else{
    ventilationStatus = "stopped_error";
  }
  preferences.putInt("vent_percent", current_ventilation_percent);
  publishVentilationStatus();
  saveVentilationSettings();
}


void ventilation_control_open_percent(int target_percent) {
  if(cooling_system_active == true){
    Serial.println("Cooling system is active, turning off cooling system");
    cooling_system_stop();
  }
  // Validate input range
  if (target_percent < 0) target_percent = 0;
  if (target_percent > 100) target_percent = 100;
  
  // Calculate the relative movement needed
  int relative_change = target_percent - current_ventilation_percent;
  
  if (relative_change == 0) {
    // No change needed
    Serial.println("Ventilation already at " + String(current_ventilation_percent) + "%");
    publishVentilationStatus();
    return;
  }
  
  // Calculate duration based on the absolute value of relative change
  int ventilation_open_interval = abs(relative_change) * ventilation_open_coefficient;
  
  Serial.println("Current position: " + String(current_ventilation_percent) + 
                 "%, Target: " + String(target_percent) + 
                 "%, Relative change: " + String(relative_change) + "%");
  
  if (relative_change < 0) {
      Serial.println("Ventilation closing from " + String(current_ventilation_percent) + 
                    "% to " + String(target_percent) + "% (Moving " + 
                    String(abs(relative_change)) + "%)");
      // Partial close
      ventilation_control_close(); // Start closing motor
      // Set up variables for non-blocking operation
      ventilation_start_time = millis();
      ventilation_open_duration = ventilation_open_interval;
      ventilation_percent_target = target_percent;
      ventilation_in_progress = true;
      } else {

      Serial.println("Ventilation opening from " + String(current_ventilation_percent) + 
                    "% to " + String(target_percent) + "% (Moving " + 
                    String(abs(relative_change)) + "%)");
      // Partial open
      ventilation_control_open(); // Start opening motor
      // Set up variables for non-blocking operation
      ventilation_start_time = millis();
      ventilation_open_duration = ventilation_open_interval;
      ventilation_percent_target = target_percent;
      ventilation_in_progress = true;
    }
}



void publishVentilationStatus(){
  StaticJsonDocument<200> doc;
  doc["status"] = ventilationStatus;
  doc["percent"] = current_ventilation_percent;
  doc["auto_control"] = ventilation_control_auto_state;
  doc["greenhouse_id"] = greenhouse_id;
  
  char buffer[200];
  serializeJson(doc, buffer);
  
  sendMQTTData(topic_ventilation_status, buffer);
}

// void ventilation_control_auto(){
//   if(ventilation_control_auto_state){
//       if(temperature > ventilation_temperature_setpoint){
//         if(ventilationStatus == "closed"){
//           Serial.println("Temp is Hight Turning Opening ventilation");
//           ventilation_control_open_percent(25);
//       }else {
//         Serial.println("Temp is Hight but Ventilation turned on Du nothing");
//        }
//     }else if(temperature < ventilation_temperature_setpoint){
//       if(ventilationStatus == "open"){
//         Serial.println("Temp is Low Turning closing ventilation");
//         Serial.println(temperature);
//         Serial.println(ventilation_temperature_setpoint);
//         ventilation_control_close();
//       }else {
//         Serial.println("Temp is low but Ventilation turned off Du nothing");
//        }
//     }
//   }
// } 

void ventilation_control_auto(){
  if(ventilation_control_auto_state){
    if(temperature > ventilation_temperature_setpoint && ventilationStatus == "closed" || ventilationStatus == "stopped"){
      ventilation_control_open_percent(25);
    }else if(temperature > ventilation_temperature_setpoint + 2){
      ventilation_control_open_percent(50);
    }else if(temperature > ventilation_temperature_setpoint + 4){
      ventilation_control_open_percent(75);
    }else if(temperature > ventilation_temperature_setpoint + 6){
      ventilation_control_open_percent(100);
    }else if(temperature < ventilation_temperature_setpoint){
      ventilation_control_open_percent(0);
    }else if(temperature < ventilation_emergency_off_temperature){
      ventilation_control_open_percent(0);
    }
  } 
}


// Function to save ventilation settings
void saveVentilationSettings() {
  preferences.putFloat("temp_setpoint", ventilation_temperature_setpoint);
  preferences.putInt("vent_percent", current_ventilation_percent);
  preferences.putBool("auto_control", ventilation_control_auto_state);
  preferences.putInt("vent_coefficient", ventilation_open_coefficient);
  preferences.putFloat("ventilation_emergency_off_temperature", ventilation_emergency_off_temperature);
}

void setVentilationSetpoints(float temperature, int coefficient, int windSpeed, float emergency_off_temperature ){
  ventilation_temperature_setpoint = temperature;
  ventilation_open_coefficient = coefficient;
  ventilation_wind_speed_setpoint = windSpeed;
  ventilation_emergency_off_temperature = emergency_off_temperature;
  saveVentilationSettings();
  publishVentilationSetpoints();
}


void publishVentilationSetpoints(){
  StaticJsonDocument<200> doc;
  doc["temperature"] = (float)ventilation_temperature_setpoint;
  doc["coefficient"] = ventilation_open_coefficient;
  doc["wind_speed"] = ventilation_wind_speed_setpoint;
  doc["emergency_off_temperature"] = ventilation_emergency_off_temperature;
  doc["greenhouse_id"] = greenhouse_id;
  
  char buffer[200];
  serializeJson(doc, buffer);
  
  sendMQTTData(ventilation_setpoints_get, buffer);
}


//Cooling System


void startVentilator(int ventilatorIndex){
  if(ventilatorIndex < 0 || ventilatorIndex > 4) return;
  turnOnRelay(ventilatorIndex);
  if(relayStatus[ventilatorIndex-1]){
    cooling_ventilators_status[ventilatorIndex-1] = true;
    String key = "v" + String(ventilatorIndex);
    preferences.putBool(key.c_str(), true);
    Serial.println("Ventilator " + String(ventilatorIndex) + " turned on successfully");
    publishCoolingStatus();
  }else{
    Serial.println("Ventilator " + String(ventilatorIndex) + " turn on error");
  }
}

void stopVentilator(int ventilatorIndex){
  if(ventilatorIndex < 0 || ventilatorIndex > 4) return;
  turnOffRelay(ventilatorIndex);
  if(!relayStatus[ventilatorIndex-1]){
    cooling_ventilators_status[ventilatorIndex-1] = false;
    String key = "v" + String(ventilatorIndex);
    preferences.putBool(key.c_str(), false);
    Serial.println("Ventilator " + String(ventilatorIndex) + " turned off successfully");
    publishCoolingStatus();
  }else{
    Serial.println("Ventilator " + String(ventilatorIndex) + " turn off error");
  }
}

void publishCoolingSetpoints(){
  StaticJsonDocument<200> doc;
  doc["target_temperature"] = cooling_target_temperature;
  doc["emergency_off_temperature"] = cooling_emergency_off_temperature;
  doc["control_auto_state"] = cooling_control_auto_state;
  doc["client_id"] = mqtt_client_id;
  doc["greenhouse_id"] = greenhouse_id;
  
  char buffer[200];
  serializeJson(doc, buffer);
  
  sendMQTTData(topic_cooling_setpoints, buffer);
}

void publishCoolingStatus(){
  StaticJsonDocument<200> doc;
  doc["status"] = cooling_system_active;
  for(int i = 0; i < sizeof(cooling_ventilators_status)/sizeof(cooling_ventilators_status[0]); i++){
    doc["v" + String(i+1)] = cooling_ventilators_status[i];
  }
  doc["water_pump"] = cooling_water_pump_status;
  doc["cooling_control_auto_state"] = cooling_control_auto_state;
  doc["client_id"] = mqtt_client_id;
  doc["greenhouse_id"] = greenhouse_id;
  
  char buffer[200];
  serializeJson(doc, buffer);
  
  sendMQTTData(topic_cooling_status, buffer);
}


void saveCoolingSettings() {
  preferences.putFloat("cooling_target_temperature", cooling_target_temperature);
  preferences.putFloat("cooling_emergency_off_temperature", cooling_emergency_off_temperature);
  preferences.putBool("cooling_control_auto_state", cooling_control_auto_state);
  preferences.putBool("cooling_system_active", cooling_system_active);
  for(int i = 0; i < sizeof(cooling_ventilators_status)/sizeof(cooling_ventilators_status[0]); i++){
    String key = "v" + String(i+1);
    preferences.putBool(key.c_str(), cooling_ventilators_status[i]);
  }
  preferences.putBool("cooling_water_pump_status", cooling_water_pump_status);
}

void setCoolingSetpoints(float target_temperature, float emergency_off_temperature, bool control_auto_state){
  cooling_target_temperature = target_temperature;
  cooling_emergency_off_temperature = emergency_off_temperature;
  cooling_control_auto_state = control_auto_state;
  saveCoolingSettings();
  publishCoolingSetpoints();
}


void cooling_control_auto(){
  if(cooling_control_auto_state){
    if(temperature > cooling_target_temperature && cooling_system_active == false){
      cooling_system_active = true;
      cooling_system_start(25);
    }else if(temperature > cooling_target_temperature + 2){
      cooling_system_start(50);
    }else if(temperature > cooling_target_temperature + 4){
      cooling_system_start(75);
    }else if(temperature > cooling_target_temperature + 6){
      cooling_system_start(100);
    }else if(temperature < cooling_target_temperature){
      cooling_system_stop();
    }else if(temperature < cooling_emergency_off_temperature){
      cooling_system_active = false;
      cooling_system_stop();
    }
  } 
}

// Start Cooling System

// Function to start cooling system, waiting for ventilation to fully close
void cooling_system_start(int percent) {
  // Check if ventilation is currently open
  if (current_ventilation_percent > 0 || ventilation_in_progress) {
    Serial.println("Waiting for ventilation to close before starting cooling system");
    // Close ventilation first
    ventilation_control_open_percent(0);
     
    // Wait until ventilation is fully closed
    unsigned long startWaitTime = millis();
    const unsigned long maxWaitTime = 60000; // Maximum wait time: 60 seconds
    
    // Wait until both conditions are met:
    // 1. current_ventilation_percent is 0
    // 2. ventilation_in_progress is false (movement has stopped)
    while (current_ventilation_percent > 0 || ventilation_in_progress) {
      // Check for timeout
      if (millis() - startWaitTime > maxWaitTime) {
        Serial.println("Timeout waiting for ventilation to close!");
        break;
      }
      
      // Process any pending MQTT messages while waiting
      mqttClient.loop();
      delay(100);
      
      // Print status every 2 seconds
      if (millis() - startWaitTime % 2000 < 100) {
        Serial.print("Waiting for ventilation: ");
        Serial.print(current_ventilation_percent);
        Serial.print("%, in_progress: ");
        Serial.println(ventilation_in_progress ? "true" : "false");
      }
    }
    
    Serial.println("Ventilation closed successfully!");
  }
  
  // Now start cooling system with requested percentage
  Serial.print("Starting cooling system at ");
  Serial.print(percent);
  Serial.println("%");
  
  // Normalize percent value
  percent = constrain(percent, 0, 100);
  
  // Turn on cooling fans based on percentage
  if (percent >= 25) {
    startVentilator(1); // First cooling ventilator
    cooling_system_active = true;
    cooling_ventilators_status[0] = true;
  }
  
  if (percent >= 50) {
    startVentilator(1);
    startVentilator(2); // Second cooling ventilator
    cooling_ventilators_status[1] = true;
    cooling_system_active = true;
  }
  
  if (percent >= 75) {
    startVentilator(1);
    startVentilator(2);
    startVentilator(3); // Third cooling ventilator
    cooling_ventilators_status[2] = true;
    cooling_system_active = true;
  }
  
  if (percent == 100) {
    startVentilator(1);
    startVentilator(2);
    startVentilator(3);
    startVentilator(4); // Fourth cooling ventilator
    cooling_ventilators_status[3] = true;
    cooling_system_active = true;
  }
  
  // Update cooling status
  saveCoolingSettings();
  publishCoolingStatus();
}

void cooling_system_start_water_pump(){
  turnOnRelay(2);
  if(relayStatus[1]){
    cooling_water_pump_status = true;
  }
  preferences.putBool("cooling_water_pump_status", cooling_water_pump_status);
  publishCoolingStatus();
}


// Stop Cooling System

void cooling_system_stop(){
  if(cooling_system_active == true){
    cooling_system_stop_ventilators();
    cooling_system_stop_water_pump();
    cooling_system_active = false;
    saveCoolingSettings();
    publishCoolingStatus();
  }
}

void cooling_system_stop_ventilators(){
  for(int i = 0; i < sizeof(cooling_ventilators_status)/sizeof(cooling_ventilators_status[0]); i++){
    stopVentilator(i+1);
    delay(100);
    if(relayStatus[i]){
      cooling_ventilators_status[i] = true;
    }else{
      cooling_ventilators_status[i] = false;
    }
  }
  cooling_system_active = false;
  saveCoolingSettings();
  publishCoolingStatus();
}


void cooling_system_stop_water_pump(){
  turnOffRelay(2);
  if(relayStatus[1]){
    cooling_water_pump_status = false;
  }
  preferences.putBool("cooling_water_pump_status", cooling_water_pump_status);
}