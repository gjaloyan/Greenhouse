import MQTTClient from '../service/mqtt-backend.js';

// ============ CONFIGURATION ============

// Cooling definitions  
const COOLING = {
    name: 'Cooling System',
    description: 'Greenhouse cooling system',
    greenhouseId: 'Greenhouse1' // TODO: change to the actual greenhouse ID 
};

// MQTT Topics
const TOPICS = {
    COOLING_COMMAND: 'cooling/command',
    COOLING_COMMAND_VENTILATOR: 'cooling/command/ventilator/start',
    COOLING_COMMAND_VENTILATOR_STOP: 'cooling/command/ventilator/stop',
    COOLING_STATUS: 'cooling/status',
    COOLING_SETPOINTS: 'cooling/setpoints',
    COOLING_SETPOINTS_GET: 'cooling/setpoints/get',
};

// Ventilators definitions
const Ventilators = {
    'v1': { name: 'Ventilator 1', id: 1, status: false },
    'v2': { name: 'Ventilator 2', id: 2, status: false },
    'v3': { name: 'Ventilator 3', id: 3, status: false },
    'v4': { name: 'Ventilator 4', id: 4, status: false },
  };

// Timeouts in milliseconds
const TIMEOUT = {
    STATUS_WAIT: 90000  // 1 minute wait timeout
};

// Helper function to get formatted date time
function getFormattedDateTime() {
    const now = new Date();
    return now.toLocaleString('en-US', { year: 'numeric', month: '2-digit', day: '2-digit', hour: '2-digit', minute: '2-digit', second: '2-digit' });
}

// State storage
let lastCoolingState = { state: 'unknown', percent: 0, waterPump: false, ventilators: [false, false, false, false], lastUpdate: getFormattedDateTime() };
let lastCoolingSetpoints = { ventilatorsCount: 0, targetTemperature: 0, emergencyOffTemperature: 0, lastUpdate: getFormattedDateTime() };
let statusUpdatePromiseResolver = null;
let setpointsUpdatePromiseResolver = null;
let lastStatusUpdate = getFormattedDateTime();



// Check MQTT connection
async function checkConnection() {
    const status = await MQTTClient.checkGreenhouse();
    return status.success;
}

// ------- MQTT Message Handling ------- //

// Subscribe to topics
MQTTClient.subscribeToTopic(TOPICS.COOLING_STATUS)
    .then(result => console.log(`Cooling status subscription ${result.success ? 'succeeded' : 'failed'}`))
    .catch(error => console.error('Cooling status subscription error:', error.message));

MQTTClient.subscribeToTopic(TOPICS.COOLING_SETPOINTS_GET)
    .then(result => console.log(`Cooling setpoints get subscription ${result.success ? 'succeeded' : 'failed'}`))
    .catch(error => console.error('Cooling setpoints get subscription error:', error.message));

// Process status messages
MQTTClient.onMessage(TOPICS.COOLING_STATUS, (topic, message) => {
    try {
        // Skip our own messages (commands sent from this controller)
        let messageStr = message.toString().trim();
        
        // Проверяем только на команду "get", но не на название топика
        if (messageStr === "get") {
            console.log(`Ignoring get command in status topic`);
            return;
        }
        
        // Remove extra quotes if the message is a JSON string enclosed in quotes
        if (messageStr.startsWith('"') && messageStr.endsWith('"')) {
            try {
                // Try to parse as a JSON string (with escaped quotes inside)
                const unquoted = JSON.parse(messageStr);
                if (typeof unquoted === 'string' && unquoted.startsWith('{') && unquoted.endsWith('}')) {
                    messageStr = unquoted;
                    console.log('Removed extra quotes from JSON string');
                }
            } catch (e) {
                // If parsing fails, just keep the original string
                console.log('Could not parse quoted string, keeping original');
            }
        }
        
        let data;
        try {
            data = JSON.parse(messageStr);
            
            // Skip command messages (messages we sent to ESP32)
            if (data.target_greenhouse) return;
            
            // Check if message is from our greenhouse
            if (data.greenhouse_id && data.greenhouse_id !== COOLING.greenhouseId) {
                console.log(`Ignoring cooling status from different greenhouse: ${data.greenhouse_id}, expected: ${COOLING.greenhouseId}`);
                return;
            }
            
            // Process JSON status message
            // Message format: {"status":"stopped","percent":0,"greenhouse_id":"Greenhouse1"}
            const state = {
                state: data.status || 'unknown',
                percent: data.percent !== undefined ? Number(data.percent) : 0,
                waterPump: data.water_pump !== undefined ? data.water_pump : false,
                ventilators: data.ventilators !== undefined ? data.ventilators : [false, false, false, false],
                lastUpdate: getFormattedDateTime()
            };
            
            console.log(`Received cooling status: ${state.state}, ${state.percent}, ${state.waterPump}, ${state.ventilators}`);
            updateCoolingState(state);
        } catch (error) {
            // Not JSON - treat as plain text
            console.log(`Received non-JSON cooling status: ${messageStr}`);
            if (messageStr === TOPICS.COOLING_STATUS) {
                console.log('Ignoring topic name echo');
                return;
            }
            
            const formattedTime = getFormattedDateTime();
            const state = {
                state: messageStr,
                percent: messageStr === 'open' ? 100 : messageStr === 'closed' ? 0 : lastCoolingState.percent,
                lastUpdate: formattedTime
            };
            updateCoolingState(state);
        }
    } catch (error) {
        console.error('Error processing cooling status:', error.message);
    }
});

// Process setpoints messages
MQTTClient.onMessage(TOPICS.COOLING_SETPOINTS_GET, (topic, message) => {
    try {
        let messageStr = message.toString().trim();
        
        // Проверяем только на команду "get", но не на название топика
        if (messageStr === "get") {
            console.log(`Ignoring get command in setpoints topic`);
            return;
        }
        
        // Remove extra quotes if the message is a JSON string enclosed in quotes
        if (messageStr.startsWith('"') && messageStr.endsWith('"')) {
            try {
                // Try to parse as a JSON string (with escaped quotes inside)
                const unquoted = JSON.parse(messageStr);
                if (typeof unquoted === 'string' && unquoted.startsWith('{') && unquoted.endsWith('}')) {
                    messageStr = unquoted;
                    console.log('Removed extra quotes from JSON string');
                }
            } catch (e) {
                // If parsing fails, just keep the original string
                console.log('Could not parse quoted string, keeping original');
            }
        }
        
        // Try to parse as JSON
        try {
            const data = JSON.parse(messageStr);
            console.log("Setpoints data received:", data);
            
            // Skip command messages
            if (data.target_greenhouse) return;
            
            // Check if message is from our greenhouse
            if (data.greenhouse_id && data.greenhouse_id !== COOLING.greenhouseId) {
                console.log(`Ignoring cooling setpoints from different greenhouse: ${data.greenhouse_id}, expected: ${COOLING.greenhouseId}`);
                return;
            }
            
            lastCoolingSetpoints = {
                ventilatorsCount: data.ventilators_count,
                targetTemperature: data.target_temperature,
                emergencyOffTemperature: data.emergency_off_temperature,
                lastUpdate: getFormattedDateTime()
            };  
            updateCoolingSetpoints(lastCoolingSetpoints);

            console.log(`Received cooling setpoints from ESP: ventilatorsCount=${lastCoolingSetpoints.ventilatorsCount}, temp=${lastCoolingSetpoints.targetTemperature}, emergencyOffTemperature=${lastCoolingSetpoints.emergencyOffTemperature}`);
        } catch (error) {
            // Not a valid JSON
            console.log(`Received non-JSON message in cooling setpoints topic: ${messageStr}`);
            if (messageStr === TOPICS.COOLING_SETPOINTS_GET) {
                console.log('Ignoring topic name echo');
                return;
            }
        }
    } catch (error) {
        console.error('Error processing cooling setpoints:', error.message);
    }
});


// Send command to ESP
async function publishCommand(topic, command) {
    
    // Prepare message with greenhouse ID
    const message = typeof command === 'object' ? { ...command, target_greenhouse: COOLING.greenhouseId } :
                   typeof command === 'number' ? { action: command, target_greenhouse: COOLING.greenhouseId } :
                   typeof command === 'string' ? { action: command, target_greenhouse: COOLING.greenhouseId } :
                   command;
    
    const result = await MQTTClient.publishToTopic(topic, message);
    if (!result?.success) throw new Error(`Failed to publish command: ${result?.error || 'Unknown error'}`);
    return result;
}

// Update state and resolve any pending promise
function updateCoolingState(newState) {
    lastCoolingState = newState;
    lastStatusUpdate = newState.lastUpdate;
    if (statusUpdatePromiseResolver) {
        statusUpdatePromiseResolver(newState);
        statusUpdatePromiseResolver = null;
    }
}

function updateCoolingSetpoints(newState) {
    lastCoolingSetpoints = newState;
    lastStatusUpdate = newState.lastUpdate;
    if (setpointsUpdatePromiseResolver) {
        setpointsUpdatePromiseResolver(newState);
        setpointsUpdatePromiseResolver = null;
    }
}


// Get current ventilation state (cached)
async function getCoolingState() {
    // Request fresh data and wait briefly for potential response
    try {
        // Send request for updated status
        const result = await MQTTClient.publishToTopic(TOPICS.COOLING_STATUS, "get");
        if (!result?.success) throw new Error(`Failed to publish command: ${result?.error || 'Unknown error'}`);
        
        // Wait 1 second to give ESP32 time to respond
        await new Promise(resolve => setTimeout(resolve, 1000));
        
    } catch (error) {
        console.error("Failed to request status update:", error);
    }
    
    // Return state after waiting
    return {
        name: COOLING.name,
        description: COOLING.description,
        state: lastCoolingState.state,
        percent: lastCoolingState.percent,
        waterPump: lastCoolingState.waterPump,
        ventilators: lastCoolingState.ventilators,
        lastUpdate: lastCoolingState.lastUpdate,
        source: 'Greenhouse'
    };
}








// Set ventilation command
const setCoolingAuto = async (req, res) => {
    try {
        if (!await checkConnection()) {
            return res.status(503).json({ message: 'MQTT server unavailable', success: false });
        }
        if(req.body.action === "auto") {
            await publishCommand(TOPICS.COOLING_COMMAND, "auto");
            const status = await getCoolingState();
            return res.json({ message: 'Cooling command sent for auto', ...status, success: true });
        } else if(req.body.action === "manual") {
            await publishCommand(TOPICS.COOLING_COMMAND, "manual");
            const status = await getCoolingState();
            return res.json({ message: 'Cooling command sent for manual', ...status, success: true }); 
        } else {
            return res.json({ message: 'Invalid action', success: false });
        }
    } catch (error) {
        console.error('Error setting cooling command:', error);
        return res.status(503).json({ message: error.message, success: false });
    }
};



export default {
    setCoolingAuto,
    getCoolingState,
    updateCoolingSetpoints,
    updateCoolingState
};

