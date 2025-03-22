
import UserModel from '../models/user.js';
import bcrypt from 'bcrypt';
import jwt from 'jsonwebtoken';

const register = async (req, res) => {

    try{
        const password = req.body.password
        const salt = await bcrypt.genSalt(10)
        const Hash = await bcrypt.hash(password, salt)

        const doc = new UserModel({
            email: req.body.email,
            passHash: Hash,
            name: req.body.name,
            avatarUrl: req.body.avatarUrl
        })

        const user = await doc.save()
        
        const token = jwt.sign(
            {
              _id: user._id,
            },
            'secret333',
            {
              expiresIn: '30d',
            }
        );

        const {passHash, ...userData} = user._doc
        res.json({
            ...userData,
            token
        })

    }catch(err){
        if(err.code === 11000){
            return res.status(400).json({
                message: 'User alredy exits'
            })
        }
        res.status(500).json({
            message: 'Failed to register'
        })
    }

}

const login = async (req, res) => { 

    try {
        const user = await UserModel.findOne({email: req.body.email})
        if(!user){
            return res.status(400).json({
                message: 'user not found'
            })
        }

        const isValidPass = await bcrypt.compare (req.body.password, user._doc.passHash)
        if(!isValidPass){
            return res.status(400).json({
                message: 'invalid login or password'
            })
        }

        const token = jwt.sign(        
            {
              _id: user._id,
            },
            'secret333',
            {
              expiresIn: '30d',
            }
        );

        const {passHash, ...userData} = user._doc
        res.json({
            ...userData,
            token
        })

    } catch (err) {
        res.status(500).json({
            message: 'Failed to login'
        })
    }
}

const getMe = async (req, res) => { 
    try {
    const user = await UserModel.findById(req.userId);
    
    if (!user) {
        return res.status(404).json({
            message: 'User not found'
        });
    }
    
    const { passHash, ...userData } = user._doc;
    res.json(userData);

} catch (err) {
    res.status(500).json({
        message: 'Failed to get user'
    });
}
}

export default {register, login, getMe}