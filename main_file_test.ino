
#include <ModbusMaster.h>
#include <WiFi.h>          // Changed from ESP8266WiFi.h
#include <PubSubClient.h>
#include <ArduinoJson.h>
// #include <Hash.h>
#include <Adafruit_Sensor.h>

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
const char* mqtt_client_id = "ESP8266_Greenhouse";
const char* server_client_id = "ESP8266_SHT20_1";



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

//Ventilation Topics
const char* vent_command = "ventilation";
const char* topic_ventilation_status = "ventilation/status";


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

const char* ventilationStatus = "closed";
float ventilation_temperature_setpoint = 26.0;
bool ventilation_control_auto_state = true;


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
}


void loop() {

  if (!mqttClient.connected()) {
    reconnectMQTT();
  }
  mqttClient.loop();


  // Publish sensor data periodically
  unsigned long now = millis();
  if (now - lastMsg > sensorPollInterval) {
    lastMsg = now;
    readSHT20Data();
    ventilation_control_auto();
  }
}

// Function to initialize and connect to WiFi
void setup_wifi() {
  delay(10);
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}


void reconnectMQTT() {
  // Повторяем попытки подключения к MQTT-серверу до успеха
  while (!mqttClient.connected()) {
    String clientId = mqtt_client_id;
    Serial.print("Attempting MQTT connection...");
    // Пытаемся подключиться с уникальным идентификатором клиента
    if (mqttClient.connect(clientId.c_str(), mqtt_user, mqtt_password)) {
      Serial.println("connected");
      // После подключения можно публиковать сообщения или подписываться на темы
      mqttClient.publish("greenhouse/status", "greenhouse online");

      // Subscribe to topics
      mqttClient.subscribe(topic_sht20_read);
      mqttClient.subscribe(topic_bmp280_read);
      mqttClient.subscribe(topic_ldr_read);
      mqttClient.subscribe(topic_relay_command);
      mqttClient.subscribe(topic_relay_status_get);

      // Publish initial status
      publishAllRelayStatus();

    } else {
      Serial.print("failed, rc=");
      Serial.print(mqttClient.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }
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
  
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("]: ");
  Serial.println(messageStr);


    // Handle sensor read requests
  if (topicStr == topic_sht20_read && messageStr == "request") {
    publishSHT20Data();
  } 
  else if (topicStr == topic_bmp280_read && messageStr == "request") {
    // publishBMP280Data();
  }
  else if (topicStr == topic_ldr_read && messageStr == "request") {
    // publishLDRData();
  }
    // Handle relay commands
  else if (topicStr == topic_relay_command) {
    handleRelayCommand(messageStr);
  }
  // Handle status requests
  else if (topicStr == topic_relay_status_get) {
    if (messageStr == "all") {
      publishAllRelayStatus();
    } else {
      // Check if it's a specific relay request
      if (messageStr == "r1") publishRelayStatus(1);
      else if (messageStr == "r2") publishRelayStatus(2);
      else if (messageStr == "r3") publishRelayStatus(3);
      else if (messageStr == "r4") publishRelayStatus(4);
      else if (messageStr == "v1") publishVentilationStatus();
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
    Serial.println("Relay turned on successfully");
    relayStatus[relayNum - 1] = true;
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
    Serial.println("Relay turned off successfully");
    relayStatus[relayNum - 1] = false;
  } else {
    Serial.println("Error writing to relay serial port");
    relayStatus[relayNum - 1] = false;
  }
}

// Вызывается перед отправкой запроса (TX-режим)
void preTransmission() {
  digitalWrite(RS485_CONTROL_PIN, HIGH);
}
// Вызывается после отправки запроса (RX-режим)
void postTransmission() {
  digitalWrite(RS485_CONTROL_PIN, LOW);
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
      Serial.print("Modbus read error. Code: 0x");
      Serial.println(result, HEX);
      attempt++;
      // Increase delay to allow the sensor more time to respond
      delay(500);  
    }
  }

  if (!success) {
    Serial.println("Failed to read sensor data after multiple attempts.");
  }
}

void publishSHT20Data(){
      readSHT20Data();
      StaticJsonDocument<200> doc;
      doc["temperature"] = temperature;
      doc["humidity"] = humidity;
      doc["client_id"] = mqtt_client_id;

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
  }else{
    ventilationStatus = "open_error";
  }
}

void ventilation_control_close(){
  turnOffRelay(4);
  if(relayStatus[3] == false){
    ventilationStatus = "closed";
  }else{
    ventilationStatus = "closed_error";
  }
}


void ventilation_control(String command){
  if(command == "on"){
    ventilation_control_open();
  }
  else if(command == "off"){
    ventilation_control_close();
  }
  else if(command == "status"){
    publishRelayStatus(4);
    publishVentilationStatus();
  }
}

void publishVentilationStatus(){
  sendMQTTData(topic_ventilation_status, ventilationStatus);
}

void ventilation_control_auto(){
  if(ventilation_control_auto_state){
      if(temperature > ventilation_temperature_setpoint){
        if(ventilationStatus == "closed"){
          Serial.println("Temp is Hight Turning Opening ventilation");
          Serial.println(temperature);
          Serial.println(ventilation_temperature_setpoint);
          ventilation_control_open();
      }else {
        Serial.println("Temp is Hight but Ventilation turned on Du nothing");
       }
    }else if(temperature < ventilation_temperature_setpoint){
      if(ventilationStatus == "open"){
        Serial.println("Temp is Low Turning closing ventilation");
        Serial.println(temperature);
        Serial.println(ventilation_temperature_setpoint);
        ventilation_control_close();
      }else {
        Serial.println("Temp is low but Ventilation turned off Du nothing");
       }
    }
  }
} 







