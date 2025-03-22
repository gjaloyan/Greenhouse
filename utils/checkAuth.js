import jwt from 'jsonwebtoken';


export default (req, res, next) => {
    const token = (req.headers.authorization || '').replace(/Bearer\s?/, '')
    if(token){
        try {
            const decoded =jwt.verify(token, 'secret333')
            req.userId = decoded._id
            return next()
        } catch(err) {
            if(err.name === 'JsonWebTokenError') {
                return res.status(403).json({
                    message: 'No access',
                    status: false
                })
            } else {
                return res.status(403).json({
                    message: 'Filed to check access',
                    status: false
                })
            }
        }
    } else {
        return res.status(403).json({
            message: 'No token',
            status: false
        })
    }

}