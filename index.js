import express from "express";
import mdb from 'mongoose';
import multer from 'multer';
import cors from 'cors';

import {registerValidation, loginValidation,} from './validations/validations.js';
import {handleValidationErrors, checkAuth} from './utils/index.js';
import {UserController, SensorsController} from './controllers/index.js';


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


app.post('/auth/login', loginValidation, handleValidationErrors, UserController.login)
app.post('/auth/register', registerValidation, handleValidationErrors, UserController.register)
app.get('/auth/me', checkAuth, UserController.getMe)



// app.post('/getsensors', checkAuth,  handleValidationErrors, SensorsController.getSensors)
// app.patch('/posts/:id', checkAuth,  handleValidationErrors, PostController.update)
// app.delete('/posts/:id', checkAuth,PostController.remove);
// app.get('/posts/:id', PostController.getOne)
// app.get('/posts', PostController.getAll)


app.get('/sensors', SensorsController.getSensors)






app.listen(5555, (err) => {
    if(err){
        console.log(err)
    }
    else{
        return console.log('Server is running on port 5555')
    }
})


