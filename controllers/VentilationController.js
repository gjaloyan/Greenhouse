import relayService from './RelayController.js';

// Configuration
const VENTILATION = {
    name: 'Ventilation Fan',
    description: 'Greenhouse ventilation system',
    relayId: 'v1' // Ventilation is connected to relay 5
};

/**
 * Get ventilation status directly from relay service
 */
async function getVentilationState() {
    try {
        // Call endpoint function directly instead of using mock objects
        const relayStatus = await relayService.getSpecificRelayStatus(
            { params: { relayId: VENTILATION.relayId } }, 
            { 
                json: data => data, 
                status: () => ({ json: data => data })
            }
        );
        
        // Map relay state to ventilation state
        const ventilationState = 
            relayStatus.state?.toUpperCase() === 'ON' ? 'on' : 
            relayStatus.state?.toUpperCase() === 'OFF' ? 'off' : 'unknown';
        
        return {
            name: VENTILATION.name,
            description: VENTILATION.description,
            relayId: VENTILATION.relayId,
            state: ventilationState,
            lastUpdate: relayStatus.lastUpdate || Date.now(),
            source: 'relay-service'
        };
    } catch (error) {
        console.error('Error getting ventilation state:', error);
        return {
            name: VENTILATION.name,
            description: VENTILATION.description,
            relayId: VENTILATION.relayId,
            state: 'unknown',
            error: error.message
        };
    }
}

/**
 * Turn on ventilation
 */
const turnOnVentilation = async (req, res) => {

    if (!await relayService.checkConnection()) {
        return res.status(503).json({
            message: 'MQTT server unavailable',
            success: false
        });
    }

    try {
        // Get current status
        const status = await getVentilationState();
        
        // Check if already on
        if (status.state === 'on') {
            return res.json({
                message: 'Ventilation is already on',
                ...status,
                changed: false
            });
        }
        
        // Turn on the relay
        await relayService.sendRelayCommandOn(
            { params: { relayId: VENTILATION.relayId } }, 
            { 
                json: () => {}, 
                status: () => ({ json: () => {} })
            }
        );
        
        // Get updated status
        const updatedStatus = await getVentilationState();
        
        return res.json({
            message: 'Ventilation turned on command sent',
            ...updatedStatus,
            changed: true,
            success: updatedStatus.state === 'on'
        });
    } catch (error) {
        console.error('Error turning on ventilation:', error);
        return res.status(503).json({
            message: error.message,
            success: false
        });
    }
};

/**
 * Turn off ventilation
 */
const turnOffVentilation = async (req, res) => {

    if (!await relayService.checkConnection()) {
        return res.status(503).json({
            message: 'MQTT server unavailable',
            success: false
        });
    }

    try {
        // Get current status
        const status = await getVentilationState();
        
        // Check if already off
        if (status.state === 'off') {
            return res.json({
                message: 'Ventilation is already off',
                ...status,
                changed: false
            });
        }
        
        // Turn off the relay
        await relayService.sendRelayCommandOff(
            { params: { relayId: VENTILATION.relayId } },
            { 
                json: () => {}, 
                status: () => ({ json: () => {} })
            }
        );
        
        // Get updated status
        const updatedStatus = await getVentilationState();
        
        return res.json({
            message: 'Ventilation turned off command sent',
            ...updatedStatus,
            changed: true,
            success: updatedStatus.state === 'off'
        });
    } catch (error) {
        console.error('Error turning off ventilation:', error);
        return res.status(503).json({
            message: error.message,
            success: false
        });
    }
};

/**
 * Get ventilation status
 */
const getVentilationStatus = async (req, res) => {
    try {
        const status = await getVentilationState();
        return res.json(status);
    } catch (error) {
        console.error('Error getting ventilation status:', error);
        return res.status(500).json({
            message: error.message,
            success: false
        });
    }
};

export default {
    turnOnVentilation,
    turnOffVentilation,
    getVentilationStatus
};
