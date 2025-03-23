import MQTTClient from '../service/mqtt-backend.js';

// Новая функция для отправки команды в тему 'relay/command'
const sendRelayCommand = async (req, res) => {
    const command = req.body.command;

    if (!command) {
        return res.status(400).json({ message: 'Отсутствует команда для реле' });
    }

    try {
        const publishResult = await MQTTClient.publishToTopic('relay/command', command);
        if (publishResult.success) {
            // Подписываемся на тему 'relay/status', чтобы получать подтверждения от ESP8266
            MQTTClient.subscribeToTopic('relay/status');
            MQTTClient.onMessage('relay/status', (topic, message) => {
                console.log(`Получено сообщение от ESP8266: ${message}`);
                // Здесь вы можете обработать полученное сообщение
            });
            return res.json({ message: 'Команда для реле отправлена успешно' });
        } else {
            return res.status(500).json({ message: 'Ошибка при отправке команды для реле' });
        }
    } catch (error) {
        console.error('Ошибка при отправке команды для реле:', error);
        return res.status(500).json({ message: 'Ошибка при отправке команды для реле' });
    }
};

export default {
    sendRelayCommand
}