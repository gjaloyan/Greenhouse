import MQTT from 'mqtt';

// Configuration
const brokerUrl = 'mqtt://34.71.239.197'; // Replace with your MQTT broker URL
const options = {
    clientId: 'ESP82677', // Unique client ID
    username: 'gjaloyan', // Your username if required
    password: '26839269', // Your password if required
    clean: true // Set to false if you want to maintain session state
};

// Хранилище для обработчиков сообщений по топикам
const messageHandlers = new Map();

// Последние полученные сообщения по топикам
const lastMessages = new Map();

// Create a client instance
const client = MQTT.connect(brokerUrl, options);

// Event handlers
client.on('connect', () => {
    console.log('Connected to MQTT broker');
});

client.on('message', (topic, message) => {
    const messageStr = message.toString();
    // console.log(`Received message on ${topic}: ${messageStr}`);
    
    // Сохраняем последнее сообщение
    lastMessages.set(topic, {
        message: messageStr,
        timestamp: Date.now()
    });
    
    // Вызываем все обработчики для этого топика
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
                console.log('Message published successfully - topic: ' + topic + ', message: ' + messageStr);
                resolve({success: true, topic: topic});
            }
        });
    });
}

// Функция для регистрации обработчика сообщений для конкретного топика
function onMessage(topic, callback) {
    if (!messageHandlers.has(topic)) {
        messageHandlers.set(topic, []);
    }
    messageHandlers.get(topic).push(callback);
    
    // Подписываемся на топик, если еще не подписаны
    subscribeToTopic(topic).catch(err => {
        console.error(`Failed to subscribe to topic ${topic}:`, err);
    });
}

// Функция для получения последнего сообщения из топика
function getLastMessage(topic) {
    return lastMessages.get(topic) || null;
}

// Function to check if client is connected
function isConnected() {
    return client.connected;
}

// Export the functions
export default {
    subscribeToTopic,
    publishToTopic,
    onMessage,
    getLastMessage,
    isConnected
};