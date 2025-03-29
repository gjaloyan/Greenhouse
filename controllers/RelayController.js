import MQTTClient from '../service/mqtt-backend.js';

// ============ CONFIGURATION ============

// Relay definitions
const relayConfig = {
  'r1': { name: 'Relay 1', description: 'Relay 1', id: 1 },
  'r2': { name: 'Relay 2', description: 'Relay 2', id: 2 },
  'r3': { name: 'Relay 3', description: 'Relay 3', id: 3 },
  'r4': { name: 'Relay 4', description: 'Relay 4', id: 4 },
  'v1': { name: 'Ventilation', description: 'Ventilation Main', id: 5 }
};

// MQTT Topics
const TOPICS = {
  COMMAND: 'relay/command',
  STATUS: 'relay/status',
  STATUS_REQUEST: 'relay/status/get'
};

// Timeouts in milliseconds
const TIMEOUT = {
  VERIFICATION: 500,  // Wait time after command before verifying
  ESP_RESPONSE: 2000  // Max wait for ESP8266 response
};

// Track device-reported relay states
const relayStates = new Map(
  Object.keys(relayConfig).map(id => [id, { state: 'unknown', lastUpdate: 0 }])
);

// ============ STATE MANAGEMENT ============

function getRelayState(relayId) {
  return relayStates.get(relayId)?.state || 'unknown';
}

function updateRelayState(relayId, state) {
  if (!relayConfig[relayId]) return false;
  
  relayStates.set(relayId, {
    state: state,
    lastUpdate: Date.now()
  });
  return true;
}

// ============ MQTT COMMUNICATION ============

async function checkConnection() {
  if (MQTTClient.isConnected()) return true;
  
  try {
    if (typeof MQTTClient.reconnect === 'function') {
      const reconnected = await MQTTClient.reconnect();
      if (reconnected) {
        console.log('Reconnected to MQTT broker');
        await MQTTClient.subscribeToTopic(TOPICS.STATUS);
        return true;
      }
    }
    console.log('MQTT disconnected and reconnection failed');
    return false;
  } catch (error) {
    console.error('Connection check error:', error.message);
    return false;
  }
}

async function publishToESP(topic, message) {
  if (!await checkConnection())
    throw new Error('MQTT server unavailable');
    
  const result = await MQTTClient.publishToTopic(topic, message);
  if (!result?.success)
    throw new Error(`Failed to publish: ${result?.error || 'Unknown error'}`);
    
  return result;
}

async function requestRelayStatus(target = 'all') {
  try {
    // Clear stale data (older than 5s)
    if (target === 'all') {
      Object.keys(relayConfig).forEach(id => {
        const status = relayStates.get(id);
        if (Date.now() - status.lastUpdate > 5000) 
          status.state = 'unknown';
      });
    } else if (relayConfig[target]) {
      const status = relayStates.get(target);
      if (Date.now() - status.lastUpdate > 5000)
        status.state = 'unknown';
    }
    
    // Request fresh status
    try {
      if (await checkConnection()) {
        await publishToESP(TOPICS.STATUS_REQUEST, target);
        await new Promise(resolve => setTimeout(resolve, TIMEOUT.ESP_RESPONSE));
      }
    } catch (error) {
      console.error('Status request failed:', error.message);
    }
    
    // Return current state
    if (target === 'all') {
      return Object.keys(relayConfig).reduce((acc, id) => {
        acc[id] = getRelayState(id);
        return acc;
      }, {});
    }
    return getRelayState(target);
    
  } catch (error) {
    console.error('Status request error:', error.message);
    return target === 'all' 
      ? Object.keys(relayConfig).reduce((acc, id) => { 
          acc[id] = 'unknown'; 
          return acc; 
        }, {})
      : 'unknown';
  }
}

async function controlRelay(relayId, state) {
  try {
    // Validate relay exists
    if (!relayConfig[relayId]) 
      throw new Error(`Relay '${relayId}' not found`);

    // Format command for ESP8266
    const command = state === 'ON' ? relayId : `${relayId}_OFF`;
    
    // Try to get initial state
    const initialState = await requestRelayStatus(relayId)
      .catch(() => 'unknown');
    console.log(`Initial state of ${relayId}: ${initialState}`);
    
    // Send command
    await publishToESP(TOPICS.COMMAND, command);
    console.log(`Command sent to ${relayId}: ${state}`);
    
    // Wait for ESP to process
    await new Promise(resolve => setTimeout(resolve, TIMEOUT.VERIFICATION));
    
    // Verify state change
    const actualState = await requestRelayStatus(relayId);
    const stateMatch = actualState.toUpperCase() === state.toUpperCase();
    
    if (!stateMatch) {
      console.warn(`⚠️ State mismatch: requested ${state}, ESP reports ${actualState}`);
    } else {
      console.log(`✅ ${relayId} state confirmed: ${actualState}`);
    }
    
    // Return result with ESP-reported state
    return {
      relayId,
      name: relayConfig[relayId].name,
      id: relayConfig[relayId].id,
      state: actualState,
      requestedState: state,
      stateMatch,
      success: true,
      ...(stateMatch ? {} : { 
        warning: `State mismatch: requested ${state}, ESP reports ${actualState}` 
      })
    };
  } catch (error) {
    console.error(`Relay control failed:`, error.message);
    return {
      relayId,
      name: relayConfig[relayId]?.name || 'Unknown',
      id: relayConfig[relayId]?.id || 0,
      state: 'unknown',
      requestedState: state,
      success: false,
      error: error.message
    };
  }
}

// ============ MESSAGE HANDLING ============

// Set up subscription
console.log('Setting up relay status listener...');
MQTTClient.subscribeToTopic(TOPICS.STATUS)
  .then(result => console.log(`Subscription ${result.success ? 'succeeded' : 'failed'}`))
  .catch(error => console.error('Subscription error:', error.message));

// Handle status updates safely
MQTTClient.onMessage(TOPICS.STATUS, (topic, message) => {
  try {
    console.log(`Received relay status: ${message}`);
    
    // Try parsing as JSON
    try {
      const status = JSON.parse(message);
      if (typeof status === 'object' && !Array.isArray(status)) {
        Object.entries(status).forEach(([relayId, state]) => {
          if (updateRelayState(relayId, state)) {
            console.log(`ESP reports: ${relayId} is ${state}`);
          }
        });
        return;
      }
    } catch {
      // Not JSON, handle as text
      const text = message.toString().trim();
      
      // Check for relay ID pattern
      const relayMatch = text.match(/(r[1-5]|v1)/i);
      if (relayMatch) {
        const relayId = relayMatch[1].toLowerCase();
        
        // Determine state
        let state = "unknown";
        if (text === relayId) state = "ON";
        else if (text.includes(relayId + "_OFF")) state = "OFF";
        else if (text.includes("ON")) state = "ON";
        else if (text.includes("OFF")) state = "OFF";
        
        if (updateRelayState(relayId, state)) {
          console.log(`ESP reports: ${relayId} is ${state}`);
        }
      } 
      // Handle simple ON/OFF
      else if (text.toUpperCase() === 'ON' || text.toUpperCase() === 'OFF') {
        updateRelayState('r1', text.toUpperCase());
        console.log(`ESP reports: r1 is ${text.toUpperCase()}`);
      }
    }
  } catch (error) {
    console.error('Error processing status message:', error.message);
  }
});

// ============ HTTP CONTROLLERS ============

/**
 * Get status of all relays
 */
const getRelayStatus = async (req, res) => {
  try {
    if (!await checkConnection()) {
      return res.status(503).json({
        message: 'MQTT server unavailable',
        success: false
      });
    }
    
    const states = await requestRelayStatus('all');
    
    return res.json({
      states,
      source: 'ESP8266',
      timestamp: Date.now()
    });
  } catch (error) {
    console.error('Error getting relay status:', error.message);
    return res.status(500).json({
      message: error.message,
      success: false
    });
  }
};

/**
 * Get status of a specific relay
 */
const getSpecificRelayStatus = async (req, res) => {
  try {
    const relayId = req.params.relayId;
    
    if (!relayConfig[relayId]) {
      return res.status(404).json({
        message: `Relay '${relayId}' not found`,
        success: false
      });
    }
    
    if (!await checkConnection()) {
      return res.status(503).json({
        message: 'MQTT server unavailable',
        success: false
      });
    }
    
    const state = await requestRelayStatus(relayId);
    const lastUpdate = relayStates.get(relayId)?.lastUpdate || 0;
    
    return res.json({
      relayId,
      name: relayConfig[relayId].name,
      description: relayConfig[relayId].description,
      id: relayConfig[relayId].id,
      state,
      lastUpdate,
      source: 'ESP8266'
    });
  } catch (error) {
    console.error(`Error getting relay status:`, error.message);
    return res.status(500).json({
      message: error.message,
      success: false
    });
  }
};

/**
 * Control a relay (ON or OFF)
 */
async function controlRelayEndpoint(req, res, state) {
  try {
    const relayId = req.params.relayId || 'r1';
    
    if (!relayConfig[relayId]) {
      return res.status(404).json({
        message: `Relay '${relayId}' not found`,
        success: false
      });
    }
    
    if (!await checkConnection()) {
      return res.status(503).json({
        message: 'MQTT server unavailable',
        success: false
      });
    }
    
    const result = await controlRelay(relayId, state);
    
    if (!result.success) {
      return res.status(503).json({
        message: result.error || 'Failed to control relay',
        success: false,
        relayId,
        requestedState: state
      });
    }
    
    const response = {
      message: `Command sent to ${result.name}: ${state}. ESP reports: ${result.state}`,
      relayId,
      name: result.name,
      id: result.id,
      state: result.state,
      requestedState: result.requestedState,
      success: result.success,
      source: 'ESP8266'
    };
    
    if (result.warning) response.warning = result.warning;
    if (result.stateMatch === false) response.stateMatch = false;
    
    return res.json(response);
  } catch (error) {
    console.error(`Error controlling relay:`, error.message);
    return res.status(503).json({
      message: error.message,
      success: false
    });
  }
}

const sendRelayCommandOn = (req, res) => controlRelayEndpoint(req, res, 'ON');
const sendRelayCommandOff = (req, res) => controlRelayEndpoint(req, res, 'OFF');

/**
 * Get list of available relays
 */
const getRelayList = async (req, res) => {
  try {
    // Try to get states but don't fail if it doesn't work
    const states = await checkConnection() 
      ? await requestRelayStatus('all').catch(() => ({}))
      : {};
    
    const relays = Object.entries(relayConfig).map(([relayId, config]) => ({
      relayId,
      name: config.name,
      description: config.description,
      id: config.id,
      state: states[relayId] || getRelayState(relayId) || 'unknown',
      lastUpdate: relayStates.get(relayId)?.lastUpdate || 0
    }));
    
    return res.json({
      relays,
      count: relays.length,
      source: Object.keys(states).length > 0 ? 'ESP8266' : 'cache'
    });
  } catch (error) {
    console.error('Error getting relay list:', error.message);
    
    // Still return the list with unknown states
    const relays = Object.entries(relayConfig).map(([relayId, config]) => ({
      relayId,
      name: config.name,
      description: config.description,
      id: config.id,
      state: 'unknown'
    }));
    
    return res.json({
      relays,
      count: relays.length,
      warning: 'Could not get states from ESP8266'
    });
  }
};

// Handle uncaught promise rejections
process.on('unhandledRejection', (reason) => 
  console.error('Unhandled Promise Rejection:', reason)
);

export default {
  sendRelayCommandOn,
  sendRelayCommandOff,
  getRelayStatus,
  getSpecificRelayStatus,
  getRelayList,
  checkConnection
};       