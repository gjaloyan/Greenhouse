import mqttBackend from '../service/mqtt-backend.js';

const getGreenhouseStatus = async (req, res) => {
    const greenhouseStatus = await mqttBackend.checkGreenhouse();
    console.log(greenhouseStatus);
    res.json(greenhouseStatus);
}

export default {
    getGreenhouseStatus
}   

