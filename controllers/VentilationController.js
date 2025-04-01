import MQTTClient from '../service/mqtt-backend.js';

// Configuration
const VENTILATION = {
    name: 'Ventilation Fan',
    description: 'Greenhouse ventilation system',
    greenhouseId: 'Greenhouse1' // TODO: change to the actual greenhouse ID 
};

// MQTT Topics
const TOPICS = {
    VENTILATION_COMMAND: 'ventilation/command',
    VENTILATION_STATUS: 'ventilation/status',
    VENTILATION_SETPOINTS: 'ventilation/setpoints',
    VENTILATION_SETPOINTS_GET: 'ventilation/setpoints/get'
};

// Timeouts in milliseconds
const TIMEOUT = {
    STATUS_WAIT: 90000  // 1 minute wait timeout
};

// Helper function to get formatted date time
function getFormattedDateTime() {
    // Create date object for current time
    const now = new Date();
    
    // Add 4 hours for GMT+4
    const gmt4Date = new Date(now.getTime() + (4 * 60 * 60 * 1000));
    
    // Format as YYYY-MM-DDThh:mm:ss
    const year = gmt4Date.getUTCFullYear();
    const month = String(gmt4Date.getUTCMonth() + 1).padStart(2, '0');
    const day = String(gmt4Date.getUTCDate()).padStart(2, '0');
    const hours = String(gmt4Date.getUTCHours()).padStart(2, '0');
    const minutes = String(gmt4Date.getUTCMinutes()).padStart(2, '0');
    const seconds = String(gmt4Date.getUTCSeconds()).padStart(2, '0');
    
    return `${year}-${month}-${day}T${hours}:${minutes}:${seconds}`;
}

// State storage
let lastVentilationState = { state: 'unknown', percent: 0, lastUpdate: getFormattedDateTime() };
let lastVentilationSetpoints = { temperature: 26.0, coefficient: 500, lastUpdate: getFormattedDateTime() };
let statusUpdatePromiseResolver = null;
let lastStatusUpdate = getFormattedDateTime();

// ------- MQTT Message Handling ------- //

// Subscribe to topics
MQTTClient.subscribeToTopic(TOPICS.VENTILATION_STATUS)
    .then(result => console.log(`Ventilation status subscription ${result.success ? 'succeeded' : 'failed'}`))
    .catch(error => console.error('Ventilation subscription error:', error.message));

MQTTClient.subscribeToTopic(TOPICS.VENTILATION_SETPOINTS_GET)
    .then(result => console.log(`Ventilation setpoints get subscription ${result.success ? 'succeeded' : 'failed'}`))
    .catch(error => console.error('Ventilation setpoints get subscription error:', error.message));

// Process status messages
MQTTClient.onMessage(TOPICS.VENTILATION_STATUS, (topic, message) => {
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
            if (data.greenhouse_id && data.greenhouse_id !== VENTILATION.greenhouseId) {
                console.log(`Ignoring ventilation status from different greenhouse: ${data.greenhouse_id}, expected: ${VENTILATION.greenhouseId}`);
                return;
            }
            
            // Process JSON status message
            // Message format: {"status":"stopped","percent":0,"greenhouse_id":"Greenhouse1"}
            const state = {
                state: data.status || 'unknown',
                percent: data.percent !== undefined ? Number(data.percent) : 0,
                lastUpdate: getFormattedDateTime()
            };
            
            console.log(`Received ventilation status: ${state.state}, ${state.percent}%`);
            updateVentilationState(state);
        } catch (error) {
            // Not JSON - treat as plain text
            console.log(`Received non-JSON ventilation status: ${messageStr}`);
            if (messageStr === TOPICS.VENTILATION_STATUS) {
                console.log('Ignoring topic name echo');
                return;
            }
            
            const formattedTime = getFormattedDateTime();
            const state = {
                state: messageStr,
                percent: messageStr === 'open' ? 100 : messageStr === 'closed' ? 0 : lastVentilationState.percent,
                lastUpdate: formattedTime
            };
            updateVentilationState(state);
        }
    } catch (error) {
        console.error('Error processing ventilation status:', error.message);
    }
});

// Process setpoints messages
MQTTClient.onMessage(TOPICS.VENTILATION_SETPOINTS_GET, (topic, message) => {
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
            if (data.greenhouse_id && data.greenhouse_id !== VENTILATION.greenhouseId) {
                console.log(`Ignoring ventilation setpoints from different greenhouse: ${data.greenhouse_id}, expected: ${VENTILATION.greenhouseId}`);
                return;
            }
            
            lastVentilationSetpoints = {
                temperature: data.temperature,
                coefficient: data.coefficient,
                windSpeed: data.wind_speed,
                lastUpdate: getFormattedDateTime()
            };
            
            console.log(`Received ventilation setpoints from ESP: temp=${lastVentilationSetpoints.temperature}, coef=${lastVentilationSetpoints.coefficient}, wind=${lastVentilationSetpoints.windSpeed}`);
        } catch (error) {
            // Not a valid JSON
            console.log(`Received non-JSON message in ventilation setpoints topic: ${messageStr}`);
            if (messageStr === TOPICS.VENTILATION_SETPOINTS_GET) {
                console.log('Ignoring topic name echo');
                return;
            }
        }
    } catch (error) {
        console.error('Error processing ventilation setpoints:', error.message);
    }
});

// Update state and resolve any pending promise
function updateVentilationState(newState) {
    lastVentilationState = newState;
    lastStatusUpdate = newState.lastUpdate;
    
    if (statusUpdatePromiseResolver) {
        statusUpdatePromiseResolver(newState);
        statusUpdatePromiseResolver = null;
    }
}

// ------- Core Functions ------- //

// Check MQTT connection
async function checkConnection() {
    const status = await MQTTClient.checkGreenhouse();
    return status.success;
}

// Send command to ESP
async function publishCommand(topic, command) {
    
    // Prepare message with greenhouse ID
    const message = typeof command === 'object' ? { ...command, target_greenhouse: VENTILATION.greenhouseId } :
                   typeof command === 'number' ? { action: command, target_greenhouse: VENTILATION.greenhouseId } :
                   typeof command === 'string' ? { action: command, target_greenhouse: VENTILATION.greenhouseId } :
                   command;
    
    const result = await MQTTClient.publishToTopic(topic, message);
    if (!result?.success) throw new Error(`Failed to publish command: ${result?.error || 'Unknown error'}`);
    return result;
}

// Get current ventilation state (cached)
async function getVentilationState() {
    // Request fresh data and wait briefly for potential response
    try {
        // Send request for updated status
        const result = await MQTTClient.publishToTopic(TOPICS.VENTILATION_STATUS, "get");
        if (!result?.success) throw new Error(`Failed to publish command: ${result?.error || 'Unknown error'}`);
        
        // Wait 500ms to give ESP32 time to respond
        await new Promise(resolve => setTimeout(resolve, 500));
        
    } catch (error) {
        console.error("Failed to request status update:", error);
    }
    
    // Return state after waiting
    return {
        name: VENTILATION.name,
        description: VENTILATION.description,
        state: lastVentilationState.state,
        percent: lastVentilationState.percent,
        lastUpdate: lastVentilationState.lastUpdate,
        source: 'esp32'
    };
}

// Wait for status update from ESP
async function waitForStatusUpdate(targetPercent = null, targetState = null, timeout = TIMEOUT.STATUS_WAIT) {
    // Return fresh state if it already matches
    const freshThreshold = 2000;
    const now = new Date();
    
    // Convert lastStatusUpdate from ISO string to timestamp for comparison
    const lastUpdateTime = new Date(lastStatusUpdate).getTime();
    
    if (now.getTime() - lastUpdateTime < freshThreshold) {
        const percentMatch = targetPercent === null || lastVentilationState.percent === targetPercent;
        const stateMatch = targetState === null || lastVentilationState.state === targetState;
        
        if (percentMatch && stateMatch) {
            return lastVentilationState;
        }
    }
    
    // Wait for matching update
    return new Promise((resolve, reject) => {
        const timeoutHandler = setTimeout(() => {
            statusUpdatePromiseResolver = null;
            reject(new Error(`Timed out waiting for ventilation status update`));
        }, timeout);
        
        statusUpdatePromiseResolver = (status) => {
            const percentMatch = targetPercent === null || status.percent === targetPercent;
            const stateMatch = targetState === null || status.state === targetState;
            
            if (percentMatch && stateMatch) {
                clearTimeout(timeoutHandler);
                resolve(status);
                return true;
            }
            
            // Special case for completed states even if not exact match
            if (status.state === 'stopped' || status.state === 'open' || status.state === 'closed' || 
                status.state.includes('_error') || status.state.includes('error')) {
                clearTimeout(timeoutHandler);
                resolve(status);
                return true;
            }
            
            return false; // Keep waiting
        };
    });
}

// Set ventilation to percentage
async function setVentilationPercent(percent) {
    // Validate and normalize
    percent = Math.max(0, Math.min(100, percent));
    
    // Check current status
    const currentStatus = await getVentilationState();
    if (currentStatus.percent === percent) return currentStatus;
    
    // Clear any pending updates
    statusUpdatePromiseResolver = null;
    
    // Send command
    const commandSentTime = Date.now();
    await publishCommand(TOPICS.VENTILATION_COMMAND, percent);
    
    try {
        // Wait for stopped state or timeout
        const finalStatus = await waitForStatusUpdate(percent, 'stopped', TIMEOUT.STATUS_WAIT);
        
        return {
            name: VENTILATION.name,
            description: VENTILATION.description,
            state: finalStatus.state,
            percent: finalStatus.percent,
            lastUpdate: finalStatus.lastUpdate,
            source: 'esp32',
            confirmed: finalStatus.percent === percent,
            completed: true
        };
    } catch (timeoutError) {
        // Check if we got any update at all
        if (lastStatusUpdate > commandSentTime) {
            return {
                name: VENTILATION.name,
                description: VENTILATION.description,
                state: lastVentilationState.state,
                percent: lastVentilationState.percent,
                targetPercent: percent,
                lastUpdate: lastVentilationState.lastUpdate,
                source: 'esp32',
                inProgress: true,
                warning: "Partial update received, movement may still be in progress"
            };
        }
        
        // No update received
        return {
            ...currentStatus,
            targetPercent: percent,
            message: "Command sent but ESP32 status update timed out"
        };
    }
}

// Update ventilation setpoints
async function setVentilationSetpoints(temperature, coefficient, windSpeed) {
    
    // Validate inputs
    const temp = parseFloat(temperature);
    const coef = parseInt(coefficient);
    const wSpeed = parseInt(windSpeed);
    
    if (isNaN(temp) || temp < 0 || temp > 40) throw new Error('Invalid temperature (0-40°C)');
    if (isNaN(coef) || coef < 100 || coef > 2000) throw new Error('Invalid coefficient (100-2000)');
    
    // Send command
    await publishCommand(TOPICS.VENTILATION_SETPOINTS, { 
        ventilation_setpoint_temperature: temp,
        ventilation_setpoint_coefficient: coef,
        ventilation_setpoint_wind_speed: wSpeed
    });
    
    return {
        temperature: temp,
        coefficient: coef,
        windSpeed: wSpeed,
        name: VENTILATION.name,
        description: VENTILATION.description,
        updated: true
    };
}

// ------- API Controllers ------- //

// Set ventilation percentage
const setVentilation = async (req, res) => {
    try {
        if (!await checkConnection()) {
            return res.status(503).json({ message: 'MQTT server unavailable', success: false });
        }
        
        // Get and validate percent
        const targetPercent = parseInt(req.params.percent || req.query.percent || req.body.percent);
        if (isNaN(targetPercent) || targetPercent < 0 || targetPercent > 100) {
            return res.status(400).json({ message: 'Invalid percentage (0-100)', success: false });
        }
        
        // Get current status and check if already at target
        const status = await getVentilationState();

        if (status.percent === targetPercent) {
            return res.json({
                message: `Ventilation already at ${targetPercent}%`,
                ...status,
                changed: false,
                success: true
            });
        }
        
        // Set percentage and wait for confirmation
        const updatedStatus = await setVentilationPercent(targetPercent);
        
        const message = updatedStatus.confirmed ? `Ventilation adjusted to ${targetPercent}%` :
                       updatedStatus.completed ? `Ventilation movement completed at ${updatedStatus.percent}%` :
                       updatedStatus.inProgress ? `Ventilation moving to ${targetPercent}%` :
                       `Ventilation command sent for ${targetPercent}%`;
        
        return res.json({
            message,
            ...updatedStatus,
            changed: true,
            success: true
        });
    } catch (error) {
        console.error('Error setting ventilation:', error);
        return res.status(503).json({ message: error.message, success: false });
    }
};

// Get ventilation status
const getVentilationStatus = async (req, res) => {
    try {
        if (!await checkConnection()) {
            return res.status(503).json({ message: 'MQTT server unavailable', success: false });
        }
        const status = await getVentilationState();
        return res.json(status);
    } catch (error) {
        console.error('Error getting ventilation status:', error);
        return res.status(500).json({ message: error.message, success: false });
    }
};

// Get setpoints
const getVentilationSetpoints = async (req, res) => {
    try {
        if (!await checkConnection()) {
            return res.status(503).json({ message: 'MQTT server unavailable', success: false });
        }
        // Request fresh data from ESP32
        try {
            // Send request for updated setpoints
            const result = await MQTTClient.publishToTopic(TOPICS.VENTILATION_SETPOINTS_GET, "get");
            if (!result?.success) throw new Error(`Failed to publish setpoints request: ${result?.error || 'Unknown error'}`);
            
            // Wait 500ms to give ESP32 time to respond
            await new Promise(resolve => setTimeout(resolve, 500));
            
        } catch (error) {
            console.error("Failed to request setpoints update:", error);
        }
        
        // Return cached setpoints after waiting
        const setpoints = {
            ...lastVentilationSetpoints,
            name: VENTILATION.name,
            description: VENTILATION.description
        };
        
        return res.json(setpoints);
    } catch (error) {
        console.error('Error getting ventilation setpoints:', error);
        return res.status(500).json({ message: error.message, success: false });
    }
};

// Update setpoints
const updateVentilationSetpoints = async (req, res) => {
    try {
        if (!await checkConnection()) {
            return res.status(503).json({ message: 'MQTT server unavailable', success: false });
        }
        
        const { temperature, coefficient, windSpeed } = req.body;
        if (temperature === undefined || coefficient === undefined || windSpeed === undefined) {
            return res.status(400).json({ message: 'Temperature, coefficient and wind speed required', success: false });
        }
        await setVentilationSetpoints(temperature, coefficient, windSpeed);
        // Wait 500ms to give ESP32 time to respond
        await new Promise(resolve => setTimeout(resolve, 500));

        const setpoints = {
            ...lastVentilationSetpoints,
            name: VENTILATION.name,
            description: VENTILATION.description
        };
        return res.json({ ...setpoints, success: true });
    } catch (error) {
        console.error('Error updating ventilation setpoints:', error);
        return res.status(400).json({ message: error.message, success: false });
    }
};

export default {
    getVentilationStatus,
    setVentilation,
    getVentilationSetpoints,
    updateVentilationSetpoints
};
