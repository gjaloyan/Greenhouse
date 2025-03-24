#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <DHT.h>

// WiFi credentials
const char* ssid = "YOUR_WIFI_SSID";
const char* password = "YOUR_WIFI_PASSWORD";

// MQTT Broker settings
const char* mqtt_server = "34.71.239.197";
const int mqtt_port = 1883;
const char* mqtt_user = "gjaloyan";
const char* mqtt_password = "26839269";
const char* mqtt_client_id = "ESP8266_Greenhouse";

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

// Define pins
#define DHTPIN 2      // DHT sensor pin (for SHT20 simulation)
#define DHTTYPE DHT22 // DHT sensor type
#define LDR_PIN A0    // Analog pin for LDR
#define RELAY1_PIN 5  // Relay 1 control pin
#define RELAY2_PIN 4  // Relay 2 control pin
#define RELAY3_PIN 14 // Relay 3 control pin
#define RELAY4_PIN 12 // Relay 4 control pin

// Objects
WiFiClient espClient;
PubSubClient client(espClient);
DHT dht(DHTPIN, DHTTYPE);

// Variables
unsigned long lastMsg = 0;
unsigned long sensorPollInterval = 30000; // 30 seconds
bool relayStatus[4] = {false, false, false, false};

// Setup function
void setup() {
  // Initialize serial
  Serial.begin(115200);
  
  // Initialize pins
  pinMode(RELAY1_PIN, OUTPUT);
  pinMode(RELAY2_PIN, OUTPUT);
  pinMode(RELAY3_PIN, OUTPUT);
  pinMode(RELAY4_PIN, OUTPUT);
  
  // Set initial relay states
  digitalWrite(RELAY1_PIN, LOW);
  digitalWrite(RELAY2_PIN, LOW);
  digitalWrite(RELAY3_PIN, LOW);
  digitalWrite(RELAY4_PIN, LOW);
  
  // Initialize DHT sensor
  dht.begin();
  
  // Connect to WiFi
  setup_wifi();
  
  // Setup MQTT
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(mqtt_callback);
}

// Main loop
void loop() {
  // Maintain MQTT connection
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  // Publish sensor data periodically
  unsigned long now = millis();
  if (now - lastMsg > sensorPollInterval) {
    lastMsg = now;
    publishSensorData();
  }
}

// WiFi setup
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

// MQTT reconnect function
void reconnect() {
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    
    if (client.connect(mqtt_client_id, mqtt_user, mqtt_password)) {
      Serial.println("connected");
      
      // Subscribe to topics
      client.subscribe(topic_sht20_read);
      client.subscribe(topic_bmp280_read);
      client.subscribe(topic_ldr_read);
      client.subscribe(topic_relay_command);
      client.subscribe(topic_relay_status_get);
      
      // Publish initial status
      publishRelayStatus();
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }
}

// MQTT message callback
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
    publishBMP280Data();
  }
  else if (topicStr == topic_ldr_read && messageStr == "request") {
    publishLDRData();
  }
  // Handle relay commands
  else if (topicStr == topic_relay_command) {
    handleRelayCommand(messageStr);
  }
  // Handle status requests
  else if (topicStr == topic_relay_status_get) {
    if (messageStr == "all") {
      publishRelayStatus();
    } else {
      // Check if it's a specific relay request
      if (messageStr == "relay1") publishRelayStatus(0);
      else if (messageStr == "relay2") publishRelayStatus(1);
      else if (messageStr == "relay3") publishRelayStatus(2);
      else if (messageStr == "relay4") publishRelayStatus(3);
    }
  }
}

// Handle relay command
void handleRelayCommand(String command) {
  int relayIndex = -1;
  bool turnOn = true;
  
  // Extract relay index (1-based in command, 0-based in array)
  if (command.startsWith("relay1")) {
    relayIndex = 0;
  } else if (command.startsWith("relay2")) {
    relayIndex = 1;
  } else if (command.startsWith("relay3")) {
    relayIndex = 2;
  } else if (command.startsWith("relay4")) {
    relayIndex = 3;
  }
  
  // Check if this is an OFF command
  if (command.endsWith("_OFF")) {
    turnOn = false;
  }
  
  // Apply the command if valid
  if (relayIndex >= 0) {
    // Get physical pin
    int pin;
    switch (relayIndex) {
      case 0: pin = RELAY1_PIN; break;
      case 1: pin = RELAY2_PIN; break;
      case 2: pin = RELAY3_PIN; break;
      case 3: pin = RELAY4_PIN; break;
    }
    
    // Set pin state
    digitalWrite(pin, turnOn ? HIGH : LOW);
    relayStatus[relayIndex] = turnOn;
    
    // Update status
    publishRelayStatus(relayIndex);
  }
}

// Publish data from all sensors
void publishSensorData() {
  publishSHT20Data();
  publishBMP280Data();
  publishLDRData();
}

// Publish SHT20 (DHT) data
void publishSHT20Data() {
  float humidity = dht.readHumidity();
  float temperature = dht.readTemperature();
  
  if (isnan(humidity) || isnan(temperature)) {
    Serial.println("Failed to read from DHT sensor!");
    return;
  }
  
  // Create JSON document
  StaticJsonDocument<200> doc;
  doc["humidity"] = humidity;
  doc["temperature"] = temperature;
  
  char buffer[200];
  serializeJson(doc, buffer);
  
  // Publish
  client.publish(topic_sht20, buffer, true);
}

// Publish BMP280 data (simulated)
void publishBMP280Data() {
  // In a real implementation, read from actual BMP280 sensor
  // Here we're just simulating
  float pressure = 1013.25 + random(-50, 50) / 10.0;
  float altitude = 110.5 + random(-10, 10) / 10.0;
  float temperature = 22.5 + random(-20, 20) / 10.0;
  
  // Create JSON document
  StaticJsonDocument<200> doc;
  doc["pressure"] = pressure;
  doc["altitude"] = altitude;
  doc["temperature"] = temperature;
  
  char buffer[200];
  serializeJson(doc, buffer);
  
  // Publish
  client.publish(topic_bmp280, buffer, true);
}

// Publish LDR data
void publishLDRData() {
  int ldrValue = analogRead(LDR_PIN);
  float lightPercent = map(ldrValue, 0, 1023, 0, 100);
  
  // Create JSON document
  StaticJsonDocument<100> doc;
  doc["light"] = lightPercent;
  doc["raw"] = ldrValue;
  
  char buffer[100];
  serializeJson(doc, buffer);
  
  // Publish
  client.publish(topic_ldr, buffer, true);
}

// Publish status of all relays
void publishRelayStatus() {
  StaticJsonDocument<200> doc;
  doc["relay1"] = relayStatus[0] ? "ON" : "OFF";
  doc["relay2"] = relayStatus[1] ? "ON" : "OFF";
  doc["relay3"] = relayStatus[2] ? "ON" : "OFF";
  doc["relay4"] = relayStatus[3] ? "ON" : "OFF";
  
  char buffer[200];
  serializeJson(doc, buffer);
  
  // Publish
  client.publish(topic_relay_status, buffer, true);
}

// Publish status of a specific relay
void publishRelayStatus(int relayIndex) {
  if (relayIndex < 0 || relayIndex > 3) return;
  
  String relayId = "relay" + String(relayIndex + 1);
  String state = relayStatus[relayIndex] ? "ON" : "OFF";
  
  StaticJsonDocument<100> doc;
  doc[relayId] = state;
  
  char buffer[100];
  serializeJson(doc, buffer);
  
  // Publish
  client.publish(topic_relay_status, buffer, true);
} 