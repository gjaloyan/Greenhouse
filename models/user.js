import mdb from 'mongoose';



const UserSchema = new mdb.Schema({
    name: {
        type: String,
        required: true
    },
    email: {
        type: String,
        required: true,
        unique: true
    },
    passHash: {
        type: String,
        required: true
    },
    avatarUrl: String 
}, {
    timestamps: true
})

const User = mdb.model('User', UserSchema);

export default User;

