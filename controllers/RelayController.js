import MQTTClient from '../service/mqtt-backend.js';

// Relay configuration
const relayConfig = {
  'relay1': {
    description: 'Relay 1',
    topic: 'relay/command',
    statusTopic: 'relay/status'
  },
  'relay2': {
    description: 'Relay 2',
    topic: 'relay/command',
    statusTopic: 'relay/status'
  },
  'relay3': {
    description: 'Relay 3',
    topic: 'relay/command',
    statusTopic: 'relay/status'
  },
  'relay4': {
    description: 'Relay 4',
    topic: 'relay/command',
    statusTopic: 'relay/status'
  }
};

// Track relay statuses
const relayStatus = new Map();

// Initialize MQTT subscription for relay status
console.log('Setting up MQTT handlers for relays...');
MQTTClient.subscribeToTopic('relay/status')
    .then(result => {
        console.log(`Subscribed to relay/status: ${result.success ? 'success' : 'error'}`);
    })
    .catch(error => {
        console.error('Error subscribing to relay/status:', error);
    });

// Initialize status for all relays
Object.keys(relayConfig).forEach(relayId => {
  relayStatus.set(relayId, {
    state: 'unknown',
    lastUpdate: 0
  });
});

// Listen for status updates
MQTTClient.onMessage('relay/status', (topic, message) => {
    try {
        // First try to parse as JSON
        const status = JSON.parse(message);
        
        // If the message contains statuses for multiple relays
        if (typeof status === 'object' && !Array.isArray(status)) {
            Object.entries(status).forEach(([relayId, state]) => {
                if (relayConfig[relayId]) {
                    relayStatus.set(relayId, {
                        state: state,
                        lastUpdate: Date.now()
                    });
                }
            });
            console.log(`Relay statuses updated: ${JSON.stringify(Object.fromEntries(relayStatus))}`);
        } else {
            // Legacy format - update all relays with the same status
            Object.keys(relayConfig).forEach(relayId => {
                relayStatus.set(relayId, {
                    state: status.toString(),
                    lastUpdate: Date.now()
                });
            });
            console.log(`All relay statuses set to: ${status.toString()}`);
        }
    } catch (error) {
        // If not JSON, try to handle raw message formats
        console.log(`Raw relay status message: "${message}"`);
        const statusText = message.toString().trim();
        
        // Check if message contains a specific relay ID
        const relayMatch = statusText.match(/(relay\d+)/i);
        if (relayMatch && relayMatch[1]) {
            const relayId = relayMatch[1].toLowerCase();
            
            if (relayConfig[relayId]) {
                // Check if message indicates an ON state
                let state = "unknown";
                
                if (statusText === relayId) {
                    // Just the relay ID means ON
                    state = "ON";
                } else if (statusText.includes(relayId + "_OFF")) {
                    // RelayID_OFF indicates OFF
                    state = "OFF";
                } else if (statusText.includes("on") || statusText.includes("ON")) {
                    state = "ON";
                } else if (statusText.includes("off") || statusText.includes("OFF")) {
                    state = "OFF";
                }
                
                relayStatus.set(relayId, {
                    state: state,
                    lastUpdate: Date.now()
                });
                
                console.log(`Relay ${relayId} status updated to: ${state}`);
            }
        } else {
            // If no specific relay mentioned, check for general ON/OFF state
            let affectedRelay = null;
            
            // Special case: handle if message is just 'ON' or 'OFF'
            if (statusText.toUpperCase() === 'ON' || statusText.toUpperCase() === 'OFF') {
                // Default to relay1 for generic ON/OFF messages
                affectedRelay = 'relay1';
                relayStatus.set(affectedRelay, {
                    state: statusText.toUpperCase(),
                    lastUpdate: Date.now()
                });
                console.log(`Default relay (${affectedRelay}) set to ${statusText.toUpperCase()}`);
            } else {
                // Check for ON/OFF mentions in the message for all relays
                Object.keys(relayConfig).forEach(relayId => {
                    if (statusText.toLowerCase().includes('on')) {
                        relayStatus.set(relayId, {
                            state: 'ON',
                            lastUpdate: Date.now()
                        });
                    } else if (statusText.toLowerCase().includes('off')) {
                        relayStatus.set(relayId, {
                            state: 'OFF',
                            lastUpdate: Date.now()
                        });
                    }
                });
            }
        }
    }
});

/**
 * Send command to control a specific relay
 * @param {string} relayId - ID of the relay to control
 * @param {string} state - Desired state (ON or OFF)
 */
const controlRelay = async (relayId, state) => {
    if (!relayConfig[relayId]) {
        throw new Error(`Relay '${relayId}' not found`);
    }
    
    // For ON command, we just send the relay ID
    // For OFF command, we send relay ID with OFF suffix
    const command = state === 'ON' ? relayId : `${relayId}_OFF`;
    
    const publishResult = await MQTTClient.publishToTopic(
        relayConfig[relayId].topic, 
        command
    );
    
    console.log(`Sent relay command: ${command} to topic: ${relayConfig[relayId].topic}`);
    
    if (!publishResult.success) {
        throw new Error('Failed to send relay command');
    }
    
    // Update local status
    relayStatus.set(relayId, {
        state: state,
        lastUpdate: Date.now()
    });
    
    return {
        relayId,
        command,
        state,
        success: true
    };
};

/**
 * Send command to turn a specific relay ON
 */
const sendRelayCommandOn = async (req, res) => {
    try {
        const relayId = req.params.relayId || req.body.relayId || 'relay1';
        
        // Validate relay ID
        if (!relayConfig[relayId]) {
            return res.status(404).json({
                message: `Relay '${relayId}' not found`,
                success: false
            });
        }
        
        const result = await controlRelay(relayId, 'ON');
        
        return res.json({ 
            message: `${relayId} turned ON successfully`,
            relayId: relayId,
            state: 'ON',
            success: true
        });
    } catch (error) {
        console.error(`Error sending relay ON command:`, error);
        return res.status(500).json({ 
            message: 'Error sending relay command: ' + error.message,
            success: false
        });
    }
};

/**
 * Send command to turn a specific relay OFF
 */
const sendRelayCommandOff = async (req, res) => {
    try {
        const relayId = req.params.relayId || req.body.relayId || 'relay1';
        
        // Validate relay ID
        if (!relayConfig[relayId]) {
            return res.status(404).json({
                message: `Relay '${relayId}' not found`,
                success: false
            });
        }
        
        const result = await controlRelay(relayId, 'OFF');
        
        return res.json({ 
            message: `${relayId} turned OFF successfully`,
            relayId: relayId,
            state: 'OFF',
            success: true
        });
    } catch (error) {
        console.error(`Error sending relay OFF command:`, error);
        return res.status(500).json({ 
            message: 'Error sending relay command: ' + error.message,
            success: false
        });
    }
};

/**
 * Get current status of all relays or a specific relay
 */
const getRelayStatus = async (req, res) => {
    try {
        const relayId = req.params.relayId || req.query.relayId;
        const now = Date.now();
        
        // If specific relay was requested
        if (relayId) {
            if (!relayConfig[relayId]) {
                return res.status(404).json({
                    message: `Relay '${relayId}' not found`,
                    success: false
                });
            }
            
            const status = relayStatus.get(relayId);
            const isRecent = (now - status.lastUpdate) < 10000; // 10 seconds
            
            // Request fresh status if needed
            if (!isRecent || status.state === 'unknown') {
                await MQTTClient.publishToTopic('relay/status/get', relayId);
                // Wait briefly for response
                await new Promise(resolve => setTimeout(resolve, 2000));
            }
            
            return res.json({
                relayId: relayId,
                state: relayStatus.get(relayId).state,
                lastUpdate: relayStatus.get(relayId).lastUpdate,
                fresh: isRecent
            });
        } else {
            // Return status of all relays
            const allStates = {};
            let needsRefresh = false;
            
            Object.keys(relayConfig).forEach(id => {
                const status = relayStatus.get(id);
                const isRecent = (now - status.lastUpdate) < 10000;
                
                allStates[id] = {
                    state: status.state,
                    lastUpdate: status.lastUpdate,
                    fresh: isRecent
                };
                
                if (!isRecent || status.state === 'unknown') {
                    needsRefresh = true;
                }
            });
            
            // Request refresh of all statuses if needed
            if (needsRefresh) {
                await MQTTClient.publishToTopic('relay/status/get', 'all');
                // Wait briefly for response
                await new Promise(resolve => setTimeout(resolve, 2000));
                
                // Update the response with fresh data
                Object.keys(relayConfig).forEach(id => {
                    const status = relayStatus.get(id);
                    allStates[id] = {
                        state: status.state,
                        lastUpdate: status.lastUpdate,
                        fresh: (now - status.lastUpdate) < 3000
                    };
                });
            }
            
            return res.json({
                relays: allStates,
                timestamp: now
            });
        }
    } catch (error) {
        console.error('Error getting relay status:', error);
        return res.status(500).json({
            message: 'Error getting relay status: ' + error.message,
            success: false
        });
    }
};

/**
 * Get list of available relays
 */
const getRelayList = (req, res) => {
    const relayList = Object.entries(relayConfig).map(([id, config]) => ({
        id,
        description: config.description
    }));
    
    return res.json({
        relays: relayList,
        count: relayList.length
    });
};

export default {
    sendRelayCommandOn,
    sendRelayCommandOff,
    getRelayStatus,
    getRelayList
}       