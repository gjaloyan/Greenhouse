import mdb from 'mongoose';


const PostSchema = new mdb.Schema({
    title: {
        type: String,
        required: true
    },
    text: {
        type: String,
        required: true,
    },
    tags: {
        type: Array,
        default: []
    },
    viewsCount: {
        type: Number,
        default: 0
    },
    user: {
        type: mdb.Schema.Types.ObjectId,
        ref: 'User',
        required: true
    },
    imageUrl: String
}, {
    timestamps: true
})

const Post = mdb.model('Post', PostSchema);

export default Post;
