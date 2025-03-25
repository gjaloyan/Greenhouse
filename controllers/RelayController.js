import MQTTClient from '../service/mqtt-backend.js';

// ============ CONSTANTS ============

// Relay configuration
const relayConfig = {
  'r1': { name: 'Relay 1', description: 'Relay 1', id: 1 },
  'r2': { name: 'Relay 2', description: 'Relay 2', id: 2 },
  'r3': { name: 'Relay 3', description: 'Relay 3', id: 3 },
  'r4': { name: 'Relay 4', description: 'Relay 4', id: 4 },
  'r5': { name: 'Ventilation', description: 'Ventilation', id: 5 }
};

// MQTT Topics
const TOPICS = {
  COMMAND: 'relay/command',
  STATUS: 'relay/status',
  STATUS_REQUEST: 'relay/status/get'
};

// Response timeouts
const TIMEOUT = {
  VERIFICATION: 500,   // Wait time after sending command before checking status
  ESP_RESPONSE: 2000   // Max wait time for ESP8266 to respond
};

// ============ STATE MANAGEMENT ============

// Store latest states received from ESP8266
const relayStates = new Map();

// Initialize with unknown states
Object.keys(relayConfig).forEach(relayId => {
  relayStates.set(relayId, { state: 'unknown', lastUpdate: 0 });
});

/**
 * Get current state of a relay from local cache
 * Note: This only returns the last known state from ESP8266
 */
function getRelayState(relayId) {
  return relayStates.get(relayId)?.state || 'unknown';
}

/**
 * Update the state of a relay in local cache
 * Only used when ESP8266 reports a state change
 */
function updateRelayState(relayId, state) {
  if (relayConfig[relayId]) {
    relayStates.set(relayId, {
      state: state,
      lastUpdate: Date.now()
    });
    return true;
  }
  return false;
}

// ============ MQTT COMMUNICATION ============

/**
 * Ensure MQTT connection is active and attempt reconnection if possible
 */
async function checkConnection() {
  try {
    if (MQTTClient.isConnected()) return true;
    
    console.log('MQTT disconnected. Attempting to reconnect...');
    
    // Try to reconnect if the client provides reconnect functionality
    if (typeof MQTTClient.reconnect === 'function') {
      try {
        const reconnected = await MQTTClient.reconnect();
        if (reconnected) {
          console.log('Successfully reconnected to MQTT broker');
          
          // Re-subscribe to topics after reconnection
          try {
            await MQTTClient.subscribeToTopic(TOPICS.STATUS);
            console.log('Re-subscribed to relay status topic');
          } catch (subError) {
            console.error('Failed to re-subscribe after reconnection:', subError);
          }
          
          return true;
        }
      } catch (reconnectError) {
        console.error('Reconnection attempt failed:', reconnectError);
      }
    }
    
    console.error('MQTT disconnected and reconnection failed/unavailable');
    return false;
  } catch (error) {
    console.error('Error checking MQTT connection:', error);
    return false;
  }
}

/**
 * Safely publish a message to MQTT with error handling
 */
async function safePublish(topic, message) {
  try {
    if (!await checkConnection()) {
      throw new Error('MQTT server unavailable');
    }
    
    const result = await MQTTClient.publishToTopic(topic, message);
    if (!result || !result.success) {
      throw new Error(`Failed to publish to ${topic}: ${result?.error || 'Unknown error'}`);
    }
    
    return result;
  } catch (error) {
    console.error(`Error publishing to ${topic}:`, error);
    throw error;
  }
}

/**
 * Request updated status from ESP8266 and wait for response
 * This is the primary way to get the actual relay state from device
 */
async function requestRelayStatus(target = 'all') {
  try {
    if (!await checkConnection()) {
      throw new Error('MQTT server unavailable');
    }
    
    // Clear any existing state data that's older than 5 seconds
    if (target === 'all') {
      Object.keys(relayConfig).forEach(relayId => {
        const status = relayStates.get(relayId);
        if (Date.now() - status.lastUpdate > 5000) {
          status.state = 'unknown';
        }
      });
    } else if (relayConfig[target]) {
      const status = relayStates.get(target);
      if (Date.now() - status.lastUpdate > 5000) {
        status.state = 'unknown';
      }
    }
    
    // Request fresh status from device
    console.log(`Requesting relay status from ESP8266: ${target}`);
    
    try {
      await safePublish(TOPICS.STATUS_REQUEST, target);
      
      // Wait for ESP8266 to respond
      await new Promise(resolve => setTimeout(resolve, TIMEOUT.ESP_RESPONSE));
    } catch (publishError) {
      console.error('Failed to request relay status:', publishError);
      // Continue execution - we'll return the cached state
    }
    
    // Return the state (may still be 'unknown' if device didn't respond)
    if (target === 'all') {
      const states = {};
      Object.keys(relayConfig).forEach(relayId => {
        states[relayId] = getRelayState(relayId);
      });
      return states;
    } else {
      return getRelayState(target);
    }
  } catch (error) {
    console.error(`Error in requestRelayStatus:`, error);
    
    // Even if there's an error, return a valid response
    if (target === 'all') {
      const states = {};
      Object.keys(relayConfig).forEach(relayId => {
        states[relayId] = 'unknown';
      });
      return states;
    } else {
      return 'unknown';
    }
  }
}

/**
 * Send command to ESP8266 to control a relay and get the resulting state
 * Only the ESP8266's reported state is considered authoritative
 */
async function controlRelay(relayId, state) {
  try {
    // Validate inputs
    if (!relayConfig[relayId]) {
      throw new Error(`Relay '${relayId}' not found`);
    }
    
    if (!await checkConnection()) {
      throw new Error('MQTT server unavailable');
    }
    
    // Format command according to ESP8266 expectations (r1 for ON, r1_OFF for OFF)
    const command = state === 'ON' ? relayId : `${relayId}_OFF`;
    
    // Get initial state for comparison (but don't fail if this fails)
    let initialState = 'unknown';
    try {
      initialState = await requestRelayStatus(relayId);
      console.log(`Initial state of ${relayId} is ${initialState}`);
    } catch (stateError) {
      console.warn(`Could not get initial state: ${stateError.message}`);
    }
    
    // Send command to ESP8266
    try {
      await safePublish(TOPICS.COMMAND, command);
      console.log(`Command sent to ${relayId}: ${state}, waiting for ESP8266 to process...`);
    } catch (publishError) {
      throw new Error(`Failed to send command: ${publishError.message}`);
    }
    
    // Wait before verification to give ESP8266 time to process command
    await new Promise(resolve => setTimeout(resolve, TIMEOUT.VERIFICATION));
    
    // Request and verify the actual state from the device
    try {
      console.log(`Verifying new state of ${relayId} from ESP8266...`);
      
      // Request latest state directly from ESP8266
      const actualState = await requestRelayStatus(relayId);
      const stateMatch = (actualState.toUpperCase() === state.toUpperCase());
      
      if (!stateMatch) {
        console.warn(`⚠️ Relay state mismatch: requested ${state}, but ESP8266 reports ${actualState}`);
      } else {
        console.log(`✅ Relay ${relayId} state confirmed by ESP8266: ${actualState}`);
      }
      
      // Return full details with the ESP8266-reported state
      return {
        relayId,
        name: relayConfig[relayId].name,
        id: relayConfig[relayId].id,
        state: actualState,
        requestedState: state,
        stateMatch,
        success: true,
        ...(stateMatch ? {} : { warning: `State mismatch: requested ${state}, but ESP8266 reports ${actualState}` })
      };
    } catch (verifyError) {
      console.error(`Failed to verify relay state: ${verifyError.message}`);
      
      // Even if verification fails, return the same object format
      return {
        relayId,
        name: relayConfig[relayId].name,
        id: relayConfig[relayId].id,
        state: 'unknown',
        requestedState: state,
        stateMatch: false,
        stateVerified: false,
        success: false,
        warning: "State verification failed - ESP8266 did not respond"
      };
    }
  } catch (error) {
    console.error(`Relay control operation failed:`, error);
    
    // Return failure response instead of throwing
    return {
      relayId,
      name: relayConfig[relayId]?.name || 'Unknown Relay',
      id: relayConfig[relayId]?.id || 0,
      state: 'unknown',
      requestedState: state,
      success: false,
      error: error.message
    };
  }
}

// ============ MQTT MESSAGE HANDLING ============

// Setup MQTT subscription with proper error handling
console.log('Setting up relay status listener...');
MQTTClient.subscribeToTopic(TOPICS.STATUS)
  .then(result => {
    if (result.success) {
      console.log(`Successfully subscribed to ${TOPICS.STATUS}`);
    } else {
      console.error(`Failed to subscribe to ${TOPICS.STATUS}:`, result.error);
    }
  })
  .catch(error => {
    console.error(`Error subscribing to ${TOPICS.STATUS}:`, error);
    // Continue execution - don't let subscription failure crash the application
  });

// Create a safe wrapper for the MQTT message handler
const safeOnMessage = (topic, callback) => {
  try {
    MQTTClient.onMessage(topic, (receivedTopic, message) => {
      try {
        callback(receivedTopic, message);
      } catch (callbackError) {
        console.error(`Error in message handler for ${topic}:`, callbackError);
      }
    });
  } catch (error) {
    console.error(`Error setting up message handler for ${topic}:`, error);
  }
};

// Handle status updates from ESP8266 with error handling
safeOnMessage(TOPICS.STATUS, (topic, message) => {
  try {
    console.log(`Received relay status from ESP8266: ${message}`);
    
    // Try to parse as JSON first
    const status = JSON.parse(message);
    
    // Handle object with relay states
    if (typeof status === 'object' && !Array.isArray(status)) {
      Object.entries(status).forEach(([relayId, state]) => {
        if (updateRelayState(relayId, state)) {
          console.log(`ESP8266 reports: Relay ${relayId} is ${state}`);
        }
      });
      return;
    }
  } catch (error) {
    // Not JSON or other error, handle as text
    try {
      const text = message.toString().trim();
      
      // Check for relay ID pattern (r1, r2, etc.)
      const relayMatch = text.match(/(r[1-4])/i);
      if (relayMatch) {
        const relayId = relayMatch[1].toLowerCase();
        
        // Determine state based on message content
        let state = "unknown";
        if (text === relayId) state = "ON";
        else if (text.includes(relayId + "_OFF")) state = "OFF";
        else if (text.includes("ON")) state = "ON";
        else if (text.includes("OFF")) state = "OFF";
        
        if (updateRelayState(relayId, state)) {
          console.log(`ESP8266 reports: Relay ${relayId} is ${state}`);
        }
      } 
      // Handle simple ON/OFF messages
      else if (text.toUpperCase() === 'ON' || text.toUpperCase() === 'OFF') {
        updateRelayState('r1', text.toUpperCase());
        console.log(`ESP8266 reports: Relay r1 is ${text.toUpperCase()}`);
      }
    } catch (parseError) {
      console.error('Error processing message:', parseError);
    }
  }
});

// ============ HTTP CONTROLLERS ============

/**
 * Get status of all relays directly from ESP8266
 */
const getRelayStatus = async (req, res) => {
  try {
    if (!await checkConnection()) {
      return res.status(503).json({
        message: 'MQTT server unavailable',
        success: false
      });
    }
    
    // Always force a fresh request to the ESP8266
    const forceRefresh = req.query.refresh !== 'false';
    
    if (forceRefresh) {
      console.log('Forcing refresh of relay states from ESP8266');
    }
    
    // Get states directly from ESP8266
    const states = await requestRelayStatus('all');
    
    // Return state object
    return res.json({
      states,
      source: 'ESP8266',
      timestamp: Date.now()
    });
  } catch (error) {
    console.error('Error getting relay status:', error);
    return res.status(500).json({
      message: error.message,
      success: false
    });
  }
};

/**
 * Get status of a specific relay directly from ESP8266
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
    
    // Always request fresh status from ESP8266
    const state = await requestRelayStatus(relayId);
    const lastUpdate = relayStates.get(relayId)?.lastUpdate || 0;
    
    // Return comprehensive relay information
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
    console.error(`Error getting relay status:`, error);
    return res.status(500).json({
      message: error.message,
      success: false
    });
  }
};

/**
 * Control a relay (ON or OFF) and get status from ESP8266
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
    
    // Send command and get verified state from ESP8266
    const result = await controlRelay(relayId, state);
    
    if (!result.success) {
      return res.status(503).json({
        message: result.error || 'Failed to control relay',
        success: false,
        relayId,
        requestedState: state
      });
    }
    
    // Prepare response based on ESP8266 verification
    const response = {
      message: `Command sent to ${result.name}: ${state}. ESP8266 reports: ${result.state}`,
      relayId,
      name: result.name,
      id: result.id,
      state: result.state,
      requestedState: result.requestedState,
      success: result.success,
      source: 'ESP8266'
    };
    
    // Add warning information if needed
    if (result.warning) response.warning = result.warning;
    if (result.stateMatch === false) response.stateMatch = false;
    
    return res.json(response);
  } catch (error) {
    console.error(`Error controlling relay:`, error);
    return res.status(503).json({
      message: error.message,
      success: false
    });
  }
}

/**
 * Turn a relay ON and get status from ESP8266
 */
const sendRelayCommandOn = (req, res) => controlRelayEndpoint(req, res, 'ON');

/**
 * Turn a relay OFF and get status from ESP8266
 */
const sendRelayCommandOff = (req, res) => controlRelayEndpoint(req, res, 'OFF');

/**
 * Get list of available relays with status from ESP8266
 */
const getRelayList = async (req, res) => {
  try {
    let states = {};
    
    // Try to get fresh states, but don't fail if it doesn't work
    try {
      if (await checkConnection()) {
        states = await requestRelayStatus('all');
      }
    } catch (stateError) {
      console.error('Error refreshing relay states:', stateError);
    }
    
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
      source: states.timestamp ? 'ESP8266' : 'cache'
    });
  } catch (error) {
    console.error(`Error getting relay list:`, error);
    
    // If we can't get states from ESP8266, still return the list
    const relays = Object.entries(relayConfig).map(([relayId, config]) => ({
      relayId,
      name: config.name,
      description: config.description,
      id: config.id,
      state: 'unknown',
      error: 'Could not get state from ESP8266'
    }));
    
    return res.json({
      relays,
      count: relays.length,
      warning: 'Could not refresh states from ESP8266'
    });
  }
};

// Add uncaught exception and promise rejection handlers for safety
process.on('unhandledRejection', (reason, promise) => {
  console.error('Unhandled Promise Rejection:', reason);
  // Don't crash the application
});

export default {
  sendRelayCommandOn,
  sendRelayCommandOff,
  getRelayStatus,
  getSpecificRelayStatus,
  getRelayList,
  checkConnection
}       