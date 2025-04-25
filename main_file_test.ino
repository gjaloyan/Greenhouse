#include <ModbusMaster.h>
#include <WiFi.h>       
#include <PubSubClient.h>
#include <ArduinoJson.h>
// #include <Hash.h>
#include <Adafruit_Sensor.h>
#include <Preferences.h>
#include <esp_wifi.h>      // For esp_wifi_set_ps function
#include "driver/uart.h"

Preferences preferences;

// Для модуля MAX485, подключенного к датчикам
#define SENSOR_RS485_RX_PIN   16   // UART2 RX 
#define SENSOR_RS485_TX_PIN   17   // UART2 TX

// Для модуля MAX485, подключенного к реле
#define RELAY_RS485_RX_PIN   26    // Измените с GPIO3 на GPIO26
#define RELAY_RS485_TX_PIN   25    // Измените с GPIO1 на GPIO25

// Определение последовательных портов
HardwareSerial rs485RelaySerial(1); // UART1 для управления реле через MAX485
HardwareSerial rs485SensorSerial(2); // 


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
const char* ventilation_command_manual = "ventilation/command/manual";
const char* ventilation_command_auto = "ventilation/command/auto";

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
ModbusMaster nodeSensor; // Для датчиков
ModbusMaster nodeRelay;  // Для реле
// Variables
unsigned long lastMsg = 0;
unsigned long sensorPollInterval = 10000; // 3 seconds
bool relayStatus[32] = {false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false};
float temperature = 0.0;
float humidity    = 0.0;

// Ventilation System Variables
String ventilationStatus = "stopped";
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
int cooling_ventilators_count = 4;
bool cooling_ventilators_status[4] = {false, false, false, false};



// int cooling_ventilators_intensity_setpoint = 0;

void setup() {
  // Start Serial for debugging
  Serial.begin(9600);
  Serial.println("\n\nInitializing system...");  
  // Early WiFi initialization
  WiFi.mode(WIFI_STA);

  // Инициализация UART для датчиков
  rs485SensorSerial.begin(9600, SERIAL_8N1, SENSOR_RS485_RX_PIN, SENSOR_RS485_TX_PIN);
  
  // Инициализация UART для реле
  rs485RelaySerial.begin(9600, SERIAL_8N1, RELAY_RS485_RX_PIN, RELAY_RS485_TX_PIN);
  
    // Только один вызов WiFi инициализации
  // WiFi.mode(WIFI_STA);
  delay(100);
  setup_wifi();
  
  mqttClient.setServer(mqtt_server, mqtt_port);
  mqttClient.setCallback(mqtt_callback);
  
  // Open preferences with namespace "greenhouse"
  preferences.begin("greenhouse", false);

  // Restore saved values
  // Ventilation
  ventilation_temperature_setpoint = preferences.getFloat("temp_setpoint", 20.0); // Default 26.0
  ventilationStatus = preferences.getString("ventilation_status", "stopped");
  current_ventilation_percent = preferences.getInt("vent_percent", 0); // Default 0%
  ventilation_control_auto_state = preferences.getBool("auto_control", false); // Default false
  ventilation_wind_speed_setpoint = preferences.getInt("wind_speed", 0); // Default 0%
  ventilation_emergency_off_temperature = preferences.getFloat("ventilation_emergency_off_temperature", 0); // Default 0%
  // Cooling
  cooling_target_temperature = preferences.getFloat("cooling_target_temperature", 0); // Default 0%
  cooling_emergency_off_temperature = preferences.getFloat("cooling_emergency_off_temperature", 0); // Default 0%
  cooling_control_auto_state = preferences.getBool("cooling_control_auto_state", false); // Default false
  cooling_system_active = preferences.getBool("cooling_system_active", false); // Default false
  cooling_ventilators_count = preferences.getInt("cooling_ventilators_count", 0); // Default 0

  for (int i = 0; i < cooling_ventilators_count; i++) {
    String vKey = "v" + String(i + 1);
    cooling_ventilators_status[i] = preferences.getBool(vKey.c_str(), false);
  }


  for (int i = 0; i < sizeof(relayStatus)/sizeof(relayStatus[0]); i++) {
    String rKey = "relay" + String(i + 1);
    relayStatus[i] = preferences.getBool(rKey.c_str(), false);
  }


  // Apply loaded states to hardware
  for (int i = 0; i < sizeof(relayStatus)/sizeof(relayStatus[0]); i++) {
    if (relayStatus[i]) {
      turnOnRelay(i+1);
    } else {
      turnOffRelay(i+1);
    }
  }

  for (int i = 0; i < sizeof(cooling_ventilators_status)/sizeof(cooling_ventilators_status[0]); i++) {
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

  if (WiFi.status() != WL_CONNECTED) {
    setup_wifi();
  }

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
  
  // Disable WiFi power saving mode
  esp_wifi_set_ps(WIFI_PS_NONE);
  

  
  // Begin connection
  WiFi.begin(ssid, password);

  // Wait for connection with timeout
  int timeout_counter = 0;
  while (WiFi.status() != WL_CONNECTED /*&& timeout_counter < 50*/) {
    delay(500);
    Serial.print(".");
    // timeout_counter++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("");
    Serial.println("WiFi connected successfully");
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
      mqttClient.subscribe(topic_greenhouse_status_get);
      // Ventilation
      mqttClient.subscribe(topic_ventilation_command);
      mqttClient.subscribe(topic_ventilation_status); 
      mqttClient.subscribe(ventilation_setpoints);
      mqttClient.subscribe(ventilation_setpoints_get);
      mqttClient.subscribe(ventilation_command_manual);
      mqttClient.subscribe(ventilation_command_auto);
      // Cooling
      mqttClient.subscribe(topic_cooling_command);
      mqttClient.subscribe(topic_cooling_status);
      mqttClient.subscribe(topic_cooling_setpoints);
      mqttClient.subscribe(topic_cooling_setpoints_get);

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
  float setpointEmergencyOffTemperature = 0;

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
        setpointEmergencyOffTemperature = doc["ventilation_emergency_off_temperature"].as<float>();
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
    setVentilationSetpoints(setpointTemperature, setpointCoefficient, setpointWindSpeed, setpointEmergencyOffTemperature);
  }
  else if (topicStr == ventilation_setpoints_get && actionStr == "get") {
    publishVentilationSetpoints();
  }
  else if (topicStr == ventilation_command_manual) {
    ventilation_control_auto_state = false;
    saveVentilationSettings();
  }
  else if (topicStr == ventilation_command_auto) {
    ventilation_control_auto_state = true;
    ventilation_control_auto();
    saveVentilationSettings();
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
  uint8_t relayIndex = 1;
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
    relayStatus[relayIndex - 1] = turnOn;
    
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

// Функция для вычисления CRC16 Modbus
uint16_t ModbusCRC16(byte* buf, int len) {
  uint16_t crc = 0xFFFF;
  
  for (int pos = 0; pos < len; pos++) {
    crc ^= (uint16_t)buf[pos];  // XOR байта с текущим значением CRC
    
    for (int i = 8; i != 0; i--) {  // Для каждого бита в байте
      if ((crc & 0x0001) != 0) {    // Если младший бит = 1
        crc >>= 1;                  // Сдвиг вправо на 1 бит
        crc ^= 0xA001;              // XOR с полиномом 0xA001
      } else {                      // Иначе если младший бит = 0
        crc >>= 1;                  // Просто сдвигаем вправо
      }
    }
  }
  return crc;
}

// Улучшенная функция проверки статуса реле, соответствующая документации Waveshare
bool getRelayStatus(uint8_t relayNum) {
  if (relayNum < 1 || relayNum > 32) {
    Serial.println("Error: Relay number out of range (1-32)");
    return false;
  }

  // Modbus RTU request to read all 32 relays
  uint8_t request[8] = {0x01, 0x01, 0x00, 0x00, 0x00, 0x20, 0x3D, 0xD2};

  // Clear UART RX buffer
  while (rs485RelaySerial.available()) rs485RelaySerial.read();

  // Send request (XY485 has auto‑direction, no DE/RE control needed)
  rs485RelaySerial.write(request, 8);
  rs485RelaySerial.flush();

  // Wait for response (expect 9 bytes: 01 01 04 00 00 00 00 FB D1)
  const unsigned long RESPONSE_TIMEOUT = 1000; // 1 s
  unsigned long start = millis();
  uint8_t response[16];
  int len = 0;
  while ((millis() - start) < RESPONSE_TIMEOUT && len < 9) {
    if (rs485RelaySerial.available()) {
      response[len++] = rs485RelaySerial.read();
    }
  }

  // Debug output
  Serial.print("Bytes received: "); Serial.println(len);
  if (len) {
    Serial.print("Response: ");
    for (int i = 0; i < len; i++) {
      Serial.printf("%02X ", response[i]);
    }
    Serial.println();
  }

  // Basic validation
  if (len < 9 || response[0] != 0x01 || response[1] != 0x01 || response[2] != 0x04) {
    Serial.println("Invalid / incomplete response");
    return relayStatus[relayNum - 1]; // return cached value
  }

  // CRC check
  uint16_t recvCRC = (response[8] << 8) | response[7];
  uint16_t calcCRC = ModbusCRC16(response, 7);
  if (recvCRC != calcCRC) {
    Serial.println("CRC mismatch");
    return relayStatus[relayNum - 1];
  }

  // Parse relay bit with reversed byte order: last data byte contains relays 1‑8
  uint8_t dataByteIndex = 3 - ((relayNum - 1) / 8);  // byte3 => relays 1‑8, byte0 => relays 25‑32
  uint8_t bitPos        = (relayNum - 1) % 8;
  uint8_t dataByte      = response[3 + dataByteIndex];

  bool state = (dataByte >> bitPos) & 0x01;

  Serial.print("Data byte index="); Serial.print(dataByteIndex);
  Serial.print(" value=0x"); Serial.print(dataByte, HEX);
  Serial.print(" bitPos="); Serial.print(bitPos);
  Serial.print(" => state="); Serial.println(state);

  relayStatus[relayNum - 1] = state;

  return state;
}







// Модифицируем turnOnRelay для использования улучшенной функции проверки статуса
void turnOnRelay(int relayNum) {
  if (relayNum < 1 || relayNum > 32) return;
  
  // Отправляем команду включения
  byte commandOn[8];
  commandOn[0] = 0x01;                // Адрес устройства
  commandOn[1] = 0x05;                // Функция 05 = Write Single Coil
  commandOn[2] = 0x00;                // Старший байт адреса реле
  commandOn[3] = relayNum - 1;        // Младший байт адреса реле (0-31)
  commandOn[4] = 0xFF;                // Значение ON (старший байт)
  commandOn[5] = 0x00;                // Значение ON (младший байт)
  
  // Рассчитываем CRC
  uint16_t crc = ModbusCRC16(commandOn, 6);
  commandOn[6] = crc & 0xFF;          // Младший байт CRC
  commandOn[7] = (crc >> 8) & 0xFF;   // Старший байт CRC
  
  // Очистка буфера
  while (rs485RelaySerial.available()) {
    rs485RelaySerial.read();
  }
  
  // Режим передачи с увеличенной задержкой
  
  
  // Отправка команды включения
  rs485RelaySerial.write(commandOn, 8);
  rs485RelaySerial.flush();
  delay(10);
  
  // Предполагаем успех и обновляем внутреннее состояние
  relayStatus[relayNum - 1] = true;
  
  // Сохраняем состояние
  saveRelayState();
  
  Serial.print("Relay ");
  Serial.print(relayNum);
  Serial.println(" ON command sent");
  
  // Задержка перед проверкой статуса
  delay(200);
  
  // Проверяем фактический статус реле с помощью улучшенной функции
  bool actualStatus = getRelayStatus(relayNum);
  
  // Выводим результат сравнения ожидаемого и фактического статуса
  if (actualStatus) {
    Serial.println("Relay is verified to be ON");
  } else {
    Serial.println("WARNING: Relay should be ON but status check failed!");
  }
}

// Модифицируем turnOffRelay для использования улучшенной функции проверки статуса
void turnOffRelay(int relayNum) {
  if (relayNum < 1 || relayNum > 32) return;
  
  // Отправляем команду выключения
  byte commandOff[8];
  commandOff[0] = 0x01;                // Адрес устройства
  commandOff[1] = 0x05;                // Функция 05 = Write Single Coil
  commandOff[2] = 0x00;                // Старший байт адреса реле
  commandOff[3] = relayNum - 1;        // Младший байт адреса реле (0-31)
  commandOff[4] = 0x00;                // Значение OFF (старший байт)
  commandOff[5] = 0x00;                // Значение OFF (младший байт)
  
  // Рассчитываем CRC
  uint16_t crc = ModbusCRC16(commandOff, 6);
  commandOff[6] = crc & 0xFF;          // Младший байт CRC
  commandOff[7] = (crc >> 8) & 0xFF;   // Старший байт CRC
  
  // Очистка буфера
  while (rs485RelaySerial.available()) {
    rs485RelaySerial.read();
  }
  
  // Режим передачи с увеличенной задержкой
  
  
  // Отправка команды выключения
  rs485RelaySerial.write(commandOff, 8);
  rs485RelaySerial.flush();
  delay(10);
  
  // Режим приема с увеличенной задержкой
  
  // Предполагаем успех и обновляем внутреннее состояние
  relayStatus[relayNum - 1] = false;
  
  // Сохраняем состояние
  saveRelayState();
  
  Serial.print("Relay ");
  Serial.print(relayNum);
  Serial.println(" OFF command sent");
  
  // Задержка перед проверкой статуса
  delay(200);
  
  // Проверяем фактический статус реле с помощью улучшенной функции
  bool actualStatus = getRelayStatus(relayNum);
  
  // Выводим результат сравнения ожидаемого и фактического статуса
  if (!actualStatus) {
    Serial.println("Relay is verified to be OFF");
  } else {
    Serial.println("WARNING: Relay should be OFF but status check failed!");
  }
}


// Function to save relay state
void saveRelayState() {
    for(int i = 0; i < sizeof(relayStatus)/sizeof(relayStatus[0]); i++){
    String rKey = "relay" + String(i + 1);
    preferences.putBool(rKey.c_str(), relayStatus[i]);
  }
  // String key = "relay" + String(relayNum);
  // preferences.putBool(key.c_str(), state);
}


void readWindSpeed(){
  const int maxAttempts = 50;  // Maximum number of retry attempts
  int attempt = 0;
  bool success = false;
  while (attempt < maxAttempts && !success) {
    // Установить ID устройства (slave ID) для конкретного датчика
    // node.setServerID(2);  // Поменяйте на ID вашего датчика (обычно 1-247)
    // Чтение данных с конкретного регистра
    uint8_t result = nodeSensor.readInputRegisters(2, 1);  // Пример: адрес 2, 1 регистра
    if (result == nodeSensor.ku8MBSuccess) {
      uint16_t rawWindSpeed = nodeSensor.getResponseBuffer(0);

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


// Reads temperature & humidity from SHT20 sensor via XY485 (no ModbusMaster)
bool readSHT20Data() {
  const int maxAttempts = 10;
  int attempt = 0;
  while (attempt < maxAttempts) {
    // Build Modbus RTU request: Addr=1, Fn=0x04, Start=0x0001, Qty=0x0002
    uint8_t request[8] = {0x01, 0x04, 0x00, 0x01, 0x00, 0x02, 0x00, 0x00};
    uint16_t crc = ModbusCRC16(request, 6);
    request[6] = crc & 0xFF;
    request[7] = crc >> 8;

    // Clear RX buffer
    while (rs485SensorSerial.available()) rs485SensorSerial.read();

    // Send request (XY485 auto‑direction)
    rs485SensorSerial.write(request, 8);
    rs485SensorSerial.flush();

    // Wait for response (expect 9 bytes: 01 04 04 T_hi T_lo H_hi H_lo CRC_lo CRC_hi)
    const unsigned long timeout = 500; // ms
    uint8_t resp[16];
    int len = 0;
    unsigned long start = millis();
    while ((millis() - start) < timeout && len < 9) {
      if (rs485SensorSerial.available()) resp[len++] = rs485SensorSerial.read();
    }

    if (len == 9 && resp[0] == 0x01 && resp[1] == 0x04 && resp[2] == 0x04) {
      uint16_t recvCRC = (resp[8] << 8) | resp[7];
      uint16_t calcCRC = ModbusCRC16(resp, 7);
      if (recvCRC == calcCRC) {
        uint16_t rawT = (resp[3] << 8) | resp[4];
        uint16_t rawH = (resp[5] << 8) | resp[6];
        temperature = rawT / 10.0;
        humidity    = rawH / 10.0;
        // Serial.print("Temp: "); Serial.print(temperature);
        // Serial.print(" °C, Hum: "); Serial.print(humidity); Serial.println(" %");
        return true;
      }
    }
    Serial.println("SHT20 read failed, retry...");
    attempt++;
    delay(100);
  }
  Serial.println("Failed to read SHT20 after attempts");
  return false;
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
  doc["r5"] = relayStatus[4] ? "ON" : "OFF";
  doc["r6"] = relayStatus[5] ? "ON" : "OFF";
  doc["r7"] = relayStatus[6] ? "ON" : "OFF";
  doc["r8"] = relayStatus[7] ? "ON" : "OFF";
  doc["r9"] = relayStatus[8] ? "ON" : "OFF";
  doc["r10"] = relayStatus[9] ? "ON" : "OFF";
  doc["r11"] = relayStatus[10] ? "ON" : "OFF";
  doc["r12"] = relayStatus[11] ? "ON" : "OFF";
  doc["r13"] = relayStatus[12] ? "ON" : "OFF";
  doc["r14"] = relayStatus[13] ? "ON" : "OFF";
  doc["r15"] = relayStatus[14] ? "ON" : "OFF";
  doc["r16"] = relayStatus[15] ? "ON" : "OFF";
  doc["r17"] = relayStatus[16] ? "ON" : "OFF";
  doc["r18"] = relayStatus[17] ? "ON" : "OFF";
  doc["r19"] = relayStatus[18] ? "ON" : "OFF";
  doc["r20"] = relayStatus[19] ? "ON" : "OFF";
  doc["r21"] = relayStatus[20] ? "ON" : "OFF";
  doc["r22"] = relayStatus[21] ? "ON" : "OFF";
  doc["r23"] = relayStatus[22] ? "ON" : "OFF";
  doc["r24"] = relayStatus[23] ? "ON" : "OFF";
  doc["r25"] = relayStatus[24] ? "ON" : "OFF";
  doc["r26"] = relayStatus[25] ? "ON" : "OFF";
  doc["r27"] = relayStatus[26] ? "ON" : "OFF";
  doc["r28"] = relayStatus[27] ? "ON" : "OFF";
  doc["r29"] = relayStatus[28] ? "ON" : "OFF";
  doc["r30"] = relayStatus[29] ? "ON" : "OFF";
  doc["r31"] = relayStatus[30] ? "ON" : "OFF";
  doc["r32"] = relayStatus[31] ? "ON" : "OFF";
  doc["greenhouse_id"] = greenhouse_id;
  
  char buffer[200];
  serializeJson(doc, buffer);
  
  // Publish
  sendMQTTData(topic_relay_status, buffer);
}

// Publish status of a specific relay
void publishRelayStatus(int relayIndex) {
  if (relayIndex < 0 || relayIndex > 32) return;
  
  String state;

  String relayId = "r" + String(relayIndex);

  if(getRelayStatus(relayIndex)){
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
  turnOnRelay(2);
  if(relayStatus[1] == true){
    ventilationStatus = "opening";
    current_ventilation_percent = 100;
  }else{
    ventilationStatus = "opening_error";
  }
}

void ventilation_control_close(){
  turnOnRelay(3);
  if(relayStatus[2] == true){
    ventilationStatus = "closing";
    current_ventilation_percent = 0;
  }else{
    ventilationStatus = "closing_error";
  }
}

void ventilation_control_stop(){
  turnOffRelay(2);
  turnOffRelay(3);
  if(relayStatus[1] == false && relayStatus[2] == false){
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
      ventilationStatus = "closing";
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
      ventilationStatus = "opening";
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


void ventilation_control_auto(){
  float plus_2 = ventilation_temperature_setpoint + 2;
  float plus_4 = ventilation_temperature_setpoint + 4;
  float plus_6 = ventilation_temperature_setpoint + 6;
  if(ventilation_control_auto_state){
    if(temperature > ventilation_temperature_setpoint && temperature < plus_2 && ventilationStatus == "stopped" ){
      ventilation_control_open_percent(25);
    }else if(temperature > plus_2 && temperature < plus_4 && ventilationStatus == "stopped" ){
      ventilation_control_open_percent(50);
    }else if(temperature > plus_4 && temperature < plus_6 && ventilationStatus == "stopped" ){
      ventilation_control_open_percent(75);
    }else if(temperature > plus_6 &&  ventilationStatus == "stopped" ){
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


void startVentilator(int ventilatorIndex) {
  if(ventilatorIndex >= 1 && ventilatorIndex <= cooling_ventilators_count){
    // Вентилятор 1 = реле 5, вентилятор 2 = реле 6, и т.д.
    int relayIndex = ventilatorIndex + 2;
    turnOnRelay(relayIndex);
    delay(100);
    if(relayStatus[relayIndex-1]){ // Массив relayStatus индексируется с 0
      cooling_ventilators_status[ventilatorIndex-1] = true;
      String key = "v" + String(ventilatorIndex);
      preferences.putBool(key.c_str(), true);
      Serial.println("Ventilator " + String(ventilatorIndex) + " turned on successfully");
      publishCoolingStatus();
    }else{
      Serial.println("Ventilator " + String(ventilatorIndex) + " turn on error");
    }
  }else{
    Serial.println("Invalid ventilator index");
  }
}

void stopVentilator(int ventilatorIndex) {
  if(ventilatorIndex >= 1 && ventilatorIndex <= cooling_ventilators_count){
    // Вентилятор 1 = реле 5, вентилятор 2 = реле 6, и т.д.
    int relayIndex = ventilatorIndex + 2;
    turnOffRelay(relayIndex);
    delay(100);
    if(!relayStatus[relayIndex-1]){ // Проверяем, что реле действительно выключено
      cooling_ventilators_status[ventilatorIndex-1] = false;
      String key = "v" + String(ventilatorIndex);
      preferences.putBool(key.c_str(), false);
      Serial.println("Ventilator " + String(ventilatorIndex) + " turned off successfully");
      publishCoolingStatus();
    }else{
      Serial.println("Ventilator " + String(ventilatorIndex) + " turn off error");
    }
  }else{
    Serial.println("Invalid ventilator index");
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
  doc["ventilators"] = cooling_ventilators_status;
  doc["water_pump"] = cooling_water_pump_status;
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
    String vKey = "v" + String(i + 1);
    preferences.putBool(vKey.c_str(), cooling_ventilators_status[i]);
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
  
  // Сначала остановим все вентиляторы для определенности
  cooling_system_stop_ventilators();
  
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

void cooling_system_stop_ventilators() {
  for (int i = 0; i < sizeof(cooling_ventilators_status)/sizeof(cooling_ventilators_status[0]); i++) {
    // Вентилятор имеет индекс i+1 (от 1 до 4)
    stopVentilator(i+1); 
    delay(100);
  }
  // Убедимся, что все флаги выключены
  for (int i = 0; i < sizeof(cooling_ventilators_status)/sizeof(cooling_ventilators_status[0]); i++) {
    cooling_ventilators_status[i] = false;
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

