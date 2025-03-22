import {validationResult} from 'express-validator';

import MQTTClient from '../service/mqtt-backend.js';

// Кэш для последних полученных данных с датчиков
let lastSensorsData = null;
let lastUpdateTime = 0;
const DATA_CACHE_TIME = 5000; // Время жизни кэша в мс (5 секунд)

// Инициализируем слушатель сообщений от сенсоров
console.log('Настройка обработчика MQTT сообщений для датчиков...');

// Подписываемся на топик с датчиками и настраиваем обработчик сообщений
MQTTClient.subscribeToTopic('sensors/sht20')
  .then(result => {
    // console.log('Подписка на топик sensors/sht20:', result.success ? 'успешно' : 'ошибка');
  })
  .catch(error => {
    console.error('Ошибка при подписке на топик:', error);
  });

// Регистрируем обработчик сообщений для сохранения данных с датчиков
MQTTClient.onMessage('sensors/sht20', (topic, message) => {
  try {
    // console.log(`Получено новое сообщение от датчика: ${message}`);
    const data = JSON.parse(message);
    
    // Сохраняем данные в кэше
    lastSensorsData = data;
    lastUpdateTime = Date.now();
    
    // console.log('Данные с датчиков обновлены:', data);
  } catch (error) {
    console.error('Ошибка при обработке данных с датчиков:', error);
  }
});

// Отправляем запрос на чтение данных с датчиков
MQTTClient.publishToTopic('sensors/sht20/read', 'request')
  .then(result => {
    console.log('Запрос на чтение датчиков отправлен:', result);
  })
  .catch(error => {
    console.error('Ошибка при отправке запроса на чтение датчиков:', error);
  });

const getSensors = async (req, res) => {
    const errors = validationResult(req);
    if(!errors.isEmpty()) {
        return res.status(400).json(errors.array()[0].msg);
    }
    
    // Проверяем, подключен ли MQTT-клиент
    if (!MQTTClient.isConnected()) {
        return res.status(503).json({
            message: 'Сервер MQTT недоступен',
            status: false
        });
    }
    
    // Проверяем, есть ли свежие кэшированные данные
    const now = Date.now();
    if (lastSensorsData && (now - lastUpdateTime < DATA_CACHE_TIME)) {
        // Возвращаем кэшированные данные, если они актуальны
        return res.json({
            data: lastSensorsData,
            cached: true,
            timestamp: lastUpdateTime
        });
    }
    
    try {
        // Отправляем запрос на обновление данных через MQTT
        await MQTTClient.publishToTopic('sensors/sht20/read', 'request');
        
        // Ждем некоторое время, чтобы получить актуальные данные
        await new Promise(resolve => setTimeout(resolve, 2500));
        
        // Проверяем, получили ли мы обновленные данные
        if (lastSensorsData && (Date.now() - lastUpdateTime < 3500)) {
            return res.json({
                data: lastSensorsData,
                cached: false,
                timestamp: lastUpdateTime
            });
        } else {
            // Если данные не обновились, получаем последнее сообщение напрямую
            const lastMessage = MQTTClient.getLastMessage('sensors/sht20');
            
            if (lastMessage) {
                try {
                    const data = JSON.parse(lastMessage.message);
                    lastSensorsData = data;
                    lastUpdateTime = lastMessage.timestamp;
                    
                    return res.json({
                        data: data,
                        cached: false,
                        timestamp: lastMessage.timestamp
                    });
                } catch (error) {
                    console.error('Ошибка парсинга данных с датчиков:', error);
                }
            }
            
            // Если нет никаких данных или не удалось их распарсить
            if (lastSensorsData) {
                // Возвращаем старые данные с пометкой
                return res.json({
                    data: lastSensorsData,
                    cached: true,
                    timestamp: lastUpdateTime,
                    warning: 'Не удалось получить актуальные данные'
                });
            } else {
                return res.status(404).json({
                    message: 'Данные с датчиков не доступны',
                    status: false
                });
            }
        }
    } catch (error) {
        console.error('Ошибка получения данных с датчиков:', error);
        return res.status(500).json({
            message: error.message || 'Ошибка получения данных с датчиков',
            status: false
        });
    }
};



export default {getSensors}