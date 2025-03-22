import MQTTClient from '../service/mqtt-backend.js';

// Новая функция для отправки команды в тему 'relay/command'
const sendRelayCommand = async (req, res) => {
    const command = req.body.command;

    if (!command) {
        return res.status(400).json({ message: 'Отсутствует команда для реле' });
    }

    try {
        await MQTTClient.publishToTopic('relay/command', command);
        return res.json({ message: 'Команда для реле отправлена' });
    } catch (error) {
        console.error('Ошибка при отправке команды для реле:', error);
        return res.status(500).json({ message: 'Ошибка при отправке команды для реле' });
    }
};

export default {
    sendRelayCommand
}