import {validationResult} from 'express-validator';
import MQTTClient from '../service/mqtt-backend.js';

// Sensor configuration - easy to add new sensors
const sensorConfig = {
  'sht20': {
    topic: 'sensors/sht20',
    readTopic: 'sensors/sht20/read',
    description: 'Temperature and Humidity sensor'
  },
  // Add more sensors here
  'bmp280': {
    topic: 'sensors/bmp280',
    readTopic: 'sensors/bmp280/read',
    description: 'Pressure and Temperature sensor'
  },
  'ldr': {
    topic: 'sensors/ldr',
    readTopic: 'sensors/ldr/read',
    description: 'Light sensor'
  }
};

// Cache structure for sensor data
const sensorCache = new Map();
const DATA_CACHE_TIME = 5000; // ms (5 seconds)
const READ_TIMEOUT = 3000; // Timeout waiting for data in ms

// Setup handlers for all sensors
console.log('Setting up MQTT message handlers for all sensors...');

// Initialize sensors
Object.entries(sensorConfig).forEach(([sensorId, config]) => {
  // Initialize cache entry
  sensorCache.set(sensorId, {
    data: null,
    lastUpdateTime: 0
  });
  
  // Subscribe to sensor topic
  MQTTClient.subscribeToTopic(config.topic)
    .then(result => {
      console.log(`Subscribed to ${config.topic}: ${result.success ? 'success' : 'error'}`);
    })
    .catch(error => {
      console.error(`Error subscribing to ${config.topic}:`, error);
    });
  
  // Register message handler for the sensor
  MQTTClient.onMessage(config.topic, (topic, message) => {
    try {
      // Check if message is a control message like "request"
      if (message === "request" || message.trim() === "request") {
        console.log(`Received control message for ${sensorId}: ${message}`);
        return; // Skip processing control messages
      }
      
      // Try to parse as JSON
      const data = JSON.parse(message);
      
      // Update cache
      sensorCache.set(sensorId, {
        data: data,
        lastUpdateTime: Date.now()
      });
      
    } catch (error) {
      console.error(`Error processing data from ${sensorId}:`, error);
    }
  });
  
  // Initial read request
  MQTTClient.publishToTopic(config.readTopic, 'request')
    .then(result => {
      console.log(`Initial ${sensorId} read request sent:`, result);
    })
    .catch(error => {
      console.error(`Error sending initial read request to ${sensorId}:`, error);
    });
});

/**
 * Get data from a specific sensor
 * @param {string} sensorId - The ID of the sensor
 * @param {boolean} forceRefresh - Whether to force a refresh from the device
 * @returns {Promise<Object>} - Sensor data response
 */
const getSensorData = async (sensorId, forceRefresh = false) => {
  // Check if sensor exists
  if (!sensorConfig[sensorId]) {
    throw new Error(`Sensor '${sensorId}' not found`);
  }
  
  // Check if MQTT client is connected
  if (!MQTTClient.isConnected()) {
    throw new Error('MQTT server unavailable');
  }
  
  const cache = sensorCache.get(sensorId);
  const now = Date.now();
  
  // Return cached data if it's fresh and no force refresh requested
  if (!forceRefresh && cache.data && (now - cache.lastUpdateTime < DATA_CACHE_TIME)) {
    return {
      data: cache.data,
      cached: true,
      timestamp: cache.lastUpdateTime
    };
  }
  
  // Request fresh data
  await MQTTClient.publishToTopic(sensorConfig[sensorId].readTopic, 'request');
  
  // Wait for response
  await new Promise(resolve => setTimeout(resolve, READ_TIMEOUT));
  
  // Check if data was updated
  const updatedCache = sensorCache.get(sensorId);
  if (updatedCache.data && (Date.now() - updatedCache.lastUpdateTime < READ_TIMEOUT + 1000)) {
    return {
      data: updatedCache.data,
      cached: false,
      timestamp: updatedCache.lastUpdateTime
    };
  }
  
  // Try to get last message directly
  const lastMessage = MQTTClient.getLastMessage(sensorConfig[sensorId].topic);
  if (lastMessage) {
    try {
      const data = JSON.parse(lastMessage.message);
      // Update cache
      sensorCache.set(sensorId, {
        data: data,
        lastUpdateTime: lastMessage.timestamp
      });
      
      return {
        data: data,
        cached: false,
        timestamp: lastMessage.timestamp
      };
    } catch (error) {
      console.error(`Error parsing data from ${sensorId}:`, error);
    }
  }
  
  // Return stale data with warning if available
  if (updatedCache.data) {
    return {
      data: updatedCache.data,
      cached: true,
      timestamp: updatedCache.lastUpdateTime,
      warning: 'Could not retrieve fresh data'
    };
  }
  
  throw new Error(`No data available from sensor ${sensorId}`);
};

/**
 * HTTP controller for getting a specific sensor's data
 */
const getSensor = async (req, res) => {
  const errors = validationResult(req);
  if (!errors.isEmpty()) {
    return res.status(400).json(errors.array()[0].msg);
  }
  
  const sensorId = req.params.sensorId;
  const forceRefresh = req.query.refresh === 'true';
  
  try {
    const result = await getSensorData(sensorId, forceRefresh);
    return res.json(result);
  } catch (error) {
    console.error(`Error getting ${sensorId} data:`, error);
    return res.status(error.message.includes('not found') ? 404 : 503).json({
      message: error.message,
      status: false
    });
  }
};

/**
 * HTTP controller for getting all sensors data
 */
const getAllSensors = async (req, res) => {
  const errors = validationResult(req);
  if (!errors.isEmpty()) {
    return res.status(400).json(errors.array()[0].msg);
  }
  
  const forceRefresh = req.query.refresh === 'true';
  
  try {
    if (!MQTTClient.isConnected()) {
      return res.status(503).json({
        message: 'MQTT server unavailable',
        status: false
      });
    }
    
    // Collect data from all sensors
    const results = {};
    const sensors = Object.keys(sensorConfig);
    
    // Use Promise.allSettled to handle failures of individual sensors
    const sensorPromises = sensors.map(sensorId => 
      getSensorData(sensorId, forceRefresh)
        .then(data => {
          results[sensorId] = data;
          return { sensorId, success: true };
        })
        .catch(error => {
          results[sensorId] = { error: error.message, status: false };
          return { sensorId, success: false, error: error.message };
        })
    );
    
    await Promise.allSettled(sensorPromises);
    
    return res.json({
      sensors: results,
      timestamp: Date.now()
    });
  } catch (error) {
    console.error('Error getting all sensors data:', error);
    return res.status(500).json({
      message: error.message || 'Error retrieving sensor data',
      status: false
    });
  }
};

/**
 * Get the list of available sensors
 */
const getSensorList = (req, res) => {
  const sensorList = Object.entries(sensorConfig).map(([id, config]) => ({
    id,
    description: config.description,
    topic: config.topic
  }));
  
  return res.json({
    sensors: sensorList,
    count: sensorList.length
  });
};

export default {
  getSensor,
  getAllSensors,
  getSensorList
};