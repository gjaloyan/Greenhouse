import MQTT from 'mqtt';

// Configuration
const brokerUrl = 'mqtt://34.71.239.197'; // Replace with your MQTT broker URL
const options = {
    clientId: 'ESP82677', // Unique client ID
    username: 'gjaloyan', // Your username if required
    password: '26839269', // Your password if required
    clean: true // Set to false if you want to maintain session state
};

// Message handlers repository by topic
const messageHandlers = new Map();

// Last received messages by topic
const lastMessages = new Map();

// Create a client instance
const client = MQTT.connect(brokerUrl, options);

// Event handlers
client.on('connect', () => {
    console.log('Connected to MQTT broker');
});

client.on('message', (topic, message) => {
    const messageStr = message.toString();
    
    // Check for control messages or special formats that don't need to be logged
    const isControlMessage = messageStr === 'request' || messageStr.includes('_OFF') || 
                             messageStr.match(/^relay\d+$/);
    
    if (!isControlMessage) {
        console.log(`Received message on ${topic}: "${messageStr.substring(0, 100)}${messageStr.length > 100 ? '...' : ''}"`);
    }
    
    // Save the last message
    lastMessages.set(topic, {
        message: messageStr,
        timestamp: Date.now()
    });
    
    // Call all handlers for this topic
    if (messageHandlers.has(topic)) {
        const handlers = messageHandlers.get(topic);
        handlers.forEach(handler => {
            try {
                handler(topic, messageStr);
            } catch (error) {
                console.error(`Error in message handler for topic ${topic}:`, error);
            }
        });
    }
});

client.on('error', (err) => {
    console.error('MQTT error:', err);
});

client.on('close', () => {
    console.log('Connection to MQTT broker closed');
});

client.on('reconnect', () => {
    console.log('Attempting to reconnect to MQTT broker...');
});

// Function to subscribe to a topic
function subscribeToTopic(topic) {
    return new Promise((resolve, reject) => {
        client.subscribe(topic, (err) => {
            if (err) {
                console.error('Error subscribing to topic:', err);
                reject({success: false, error: err});
            } else {
                console.log(`Subscribed to ${topic}`);
                resolve({success: true, topic: topic});
            }
        });
    });
}

// Function to publish a message to a topic
function publishToTopic(topic, message) {
    return new Promise((resolve, reject) => {
        const messageStr = typeof message === 'string' ? message : JSON.stringify(message);
        client.publish(topic, messageStr, (err) => {
            if (err) {
                console.error('Error publishing message:', err);
                reject({success: false, error: err});
            } else {
                console.log(`Published to ${topic}: "${messageStr}"`);
                resolve({success: true, topic: topic});
            }
        });
    });
}

// Function to register a message handler for a specific topic
function onMessage(topic, callback) {
    if (!messageHandlers.has(topic)) {
        messageHandlers.set(topic, []);
    }
    messageHandlers.get(topic).push(callback);
    
    // Subscribe to the topic if not already subscribed
    subscribeToTopic(topic).catch(err => {
        console.error(`Failed to subscribe to topic ${topic}:`, err);
    });
}

// Function to get the last message from a topic
function getLastMessage(topic) {
    return lastMessages.get(topic) || null;
}

// Function to check if client is connected
function isConnected() {
    return client.connected;
}

// Function to get a list of all active subscriptions
function getActiveSubscriptions() {
    return Object.keys(client.subscriptions || {});
}

// Export the functions
export default {
    subscribeToTopic,
    publishToTopic,
    onMessage,
    getLastMessage,
    isConnected,
    getActiveSubscriptions
};