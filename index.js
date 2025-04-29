import express from "express";
import mdb from 'mongoose';
import multer from 'multer';
import cors from 'cors';

import {registerValidation, loginValidation,} from './validations/validations.js';
import {handleValidationErrors, checkAuth} from './utils/index.js';
import {UserController, SensorsController, RelayController, VentilationController, GreenhouseController, CoolingController } from './controllers/index.js';


mdb
.connect('mongodb+srv://gjaloyan:kokojambo@cluster0.yxtq3.mongodb.net/test?retryWrites=true&w=majority&appName=Cluster0')
.then(() => {
    console.log('Connected to MongoDB')
})
.catch((err) => {
    console.log('error' + err)
})

const app = express();

const storage = multer.diskStorage({
    destination: (_, __, cb) => {
        cb(null, 'uploads')
    },
    filename: (_, file, cb) => {
        cb(null, file.originalname)
    }
})

const upload = multer({storage: storage})
app.use(express.json());
app.use(cors({
    origin: 'http://localhost:3000',
    credentials: true,
}))
app.use('/uploads', express.static('uploads'))


app.get('/', (req, res) => {
    res.send('<h1>Hello World</h1>')
})  


// Auth routes
app.post('/auth/login', loginValidation, handleValidationErrors, UserController.login)
app.post('/auth/register', registerValidation, handleValidationErrors, UserController.register)
app.get('/auth/me', checkAuth, UserController.getMe)



// app.post('/getsensors', checkAuth,  handleValidationErrors, SensorsController.getSensors)
// app.patch('/posts/:id', checkAuth,  handleValidationErrors, PostController.update)
// app.delete('/posts/:id', checkAuth,PostController.remove);
// app.get('/posts/:id', PostController.getOne)
// app.get('/posts', PostController.getAll)


// Sensor routes
// Order matters! More specific routes must come before dynamic routes
app.get('/sensors', SensorsController.getSensorList)    
app.get('/sensors/data', SensorsController.getAllSensors)
app.get('/sensors/:sensorId', SensorsController.getSensor)

// Relay routes - updated for multiple relays
app.get('/relay', RelayController.getRelayList)
app.get('/relay/status/all', RelayController.getRelayStatus)
app.get('/relay/status', RelayController.getSpecificRelayStatus)
app.post('/relay/command', RelayController.RelayCommand)


// Ventilation routes
app.get('/ventilation/status', VentilationController.getVentilationStatus)
app.post('/ventilation/command', VentilationController.setVentilation)
app.get('/ventilation/setpoints/get', VentilationController.getVentilationSetpoints)
app.post('/ventilation/setpoints', VentilationController.updateVentilationSetpoints)
app.post('/ventilation/command/auto', VentilationController.setVentilationAuto)

app.post('/greenhouse/status', GreenhouseController.getGreenhouseStatus)

// // Cooling routes
// app.post('/cooling/status', CoolingController.getCoolingState)
// app.post('/cooling/setpoints', CoolingController.updateCoolingSetpoints)
// // app.post('/cooling/command', CoolingController.)
// app.post('/cooling/command/auto', CoolingController.setCoolingAuto)
// app.post('/cooling/command/ventilator/start', CoolingController.setCoolingVentilatorStart)
// app.post('/cooling/command/ventilator/stop', CoolingController.setCoolingVentilatorStop)
// app.get('/cooling/setpoints/get', CoolingController.getCoolingSetpoints)



app.listen(5555, (err) => {
    if(err){
        console.log(err)
    }
    else{
        return console.log('Server is running on port 5555')
    }
})


