# Greenhouse Monitoring and Control System

A Node.js backend system for monitoring sensors and controlling relay devices in a greenhouse environment via MQTT.

## Features

- **Sensor Monitoring**: Read data from multiple sensors connected to ESP8266
- **Relay Control**: Control multiple relays for automation
- **User Authentication**: Secure access to the system
- **RESTful API**: Easy integration with any frontend application

## API Documentation

### Authentication

- `POST /auth/login` - Login with username and password
- `POST /auth/register` - Register a new user
- `GET /auth/me` - Get current user information (requires auth token)

### Sensors

- `GET /sensors` - Get list of all available sensors
- `GET /sensors/data` - Get data from all sensors
- `GET /sensors/:sensorId` - Get data from a specific sensor

  Query parameters:
  - `refresh=true` - Force refresh data from the device instead of using cache

### Relay Control

- `GET /relay` - Get list of all available relays
- `GET /relay/status` - Get status of all relays
- `GET /relay/:relayId/status` - Get status of a specific relay
- `POST /relay/:relayId/on` - Turn a specific relay ON
- `POST /relay/:relayId/off` - Turn a specific relay OFF
- `POST /relay/on` - Turn relay1 ON (legacy)
- `POST /relay/off` - Turn relay1 OFF (legacy)

## MQTT Topics

### Sensors
- `sensors/[sensorId]` - Topic where sensor publishes its data
- `sensors/[sensorId]/read` - Topic to request sensor reading

### Relay
- `relay/command` - Topic to send relay commands
  - Send `relay1` to turn relay1 ON
  - Send `relay1_OFF` to turn relay1 OFF
  - Same for relay2, relay3, etc.
- `relay/status` - Topic where relays publish their status
- `relay/status/get` - Topic to request relay status update
  - Send `all` to get status of all relays
  - Send specific relay ID to get status of a single relay

## ESP8266 Setup

To connect your ESP8266 properly:

1. Configure the ESP8266 to connect to your MQTT broker
2. Set up sensors to publish data to the correct topics
3. Configure relays to subscribe to the command topic
4. Make sure the ESP8266 sends status updates for all relays

### ESP8266 Relay Command Handling

The ESP8266 should be programmed to process relay commands as follows:

```cpp
// Example MQTT message callback handling
void mqtt_callback(char* topic, byte* payload, unsigned int length) {
  String message = "";
  for (int i = 0; i < length; i++) {
    message += (char)payload[i];
  }
  
  if (strcmp(topic, "relay/command") == 0) {
    // Check if this is an OFF command (contains "_OFF")
    if (message.indexOf("_OFF") > 0) {
      // Extract relay ID and turn it OFF
      String relayId = message.substring(0, message.indexOf("_OFF"));
      if (relayId == "relay1") {
        digitalWrite(RELAY1_PIN, LOW);
        mqtt.publish("relay/status", "{\"relay1\":\"OFF\"}");
      } else if (relayId == "relay2") {
        digitalWrite(RELAY2_PIN, LOW);
        mqtt.publish("relay/status", "{\"relay2\":\"OFF\"}");
      } // And so on for other relays
    } else {
      // This is an ON command - the message contains just the relay ID
      if (message == "relay1") {
        digitalWrite(RELAY1_PIN, HIGH);
        mqtt.publish("relay/status", "{\"relay1\":\"ON\"}");
      } else if (message == "relay2") {
        digitalWrite(RELAY2_PIN, HIGH);
        mqtt.publish("relay/status", "{\"relay2\":\"ON\"}");
      } // And so on for other relays
    }
  } else if (strcmp(topic, "relay/status/get") == 0) {
    // Handle status request
    if (message == "all") {
      // Publish status of all relays
      String status = "{";
      status += "\"relay1\":\"" + String(digitalRead(RELAY1_PIN) == HIGH ? "ON" : "OFF") + "\",";
      status += "\"relay2\":\"" + String(digitalRead(RELAY2_PIN) == HIGH ? "ON" : "OFF") + "\"";
      status += "}";
      mqtt.publish("relay/status", status.c_str());
    } else if (message == "relay1") {
      // Publish just relay1 status
      String status = "{\"relay1\":\"" + String(digitalRead(RELAY1_PIN) == HIGH ? "ON" : "OFF") + "\"}";
      mqtt.publish("relay/status", status.c_str());
    } // And so on for other relays
  }
}
```

## Development

### Prerequisites
- Node.js v14 or higher
- MongoDB
- MQTT broker (e.g., Mosquitto)

### Installation

```bash
# Clone repository
git clone https://github.com/yourusername/greenhouse-monitor.git

# Install dependencies
npm install

# Start the server
npm start
```

### Environment Variables
- `PORT` - Server port (default: 5555)
- `MONGODB_URI` - MongoDB connection string
- `MQTT_BROKER` - MQTT broker URL

## License

MIT