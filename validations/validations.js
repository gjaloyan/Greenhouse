import {body} from 'express-validator';

export const registerValidation = [
    body('email', 'bad email format').isEmail(),
    body('password', 'password must be at least 6 characters long').isLength({min: 6}),
    body('name', 'name must be at least 3 characters long').isLength({min: 3}),
    body('avatarUrl', 'bad avatar url').optional().isURL(),
]

export const loginValidation = [
    body('email').isEmail(),
    body('password').isLength({min: 6}),
]
