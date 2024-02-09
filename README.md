<img width="525" alt="Снимок экрана 2024-02-06 в 22 46 19" src="https://github.com/MaxiEnergy/SmartPulse/assets/58640177/9c043f96-8abf-408c-b852-eb883e2d56ca">

# SmartPulse
SmartPulse - это умный пульсометр с интегрированными механизмами защиты, созданный для демонстрации методики обеспечения безопасности Интернета вещей в рамках выпускной квалификационной работы. В качестве технологии беспроводной передачи данных используется BLE

## Содержание
- [Стек технологий и компонентов](#cтек-технологий-и-компонентов)
- [Описание функционала](#описание-функционала)
- [Механизмы защиты](#механизмы-защиты)
- [Схема подключения электронных компонентов](#схема-подключения-электронных-компонентов)
- [Корпус](#корпус)
- [Выпускная квалификационная работа](#выпускная-квалификационная-работа)
- [Дополнительные материалы](#дополнительные-материалы)

## Стек технологий и компонентов

### Программная часть
- Язык программирования C
- Среда разработки ESP-IDF

### Аппаратная часть
- Pulse sensor
- OLED дисплей, SPI
- Датчик напряжения и тока CJMCU-219 на чипе INA219
- Тактовая кнопка
- Touch sensor TTP223
- Литий-полимерный аккумулятор 320 mAh
- Модуль контроллера заряда аккумулятора с защитой TP4056
- WeAct Studio ESP32-C3 Core

## Описание функционала
- Измерение частоты сердечного ритма
- Определение уровня заряда аккумулятора
- Управление отображением данных на экране устройства
- Реализация механизмов защиты из разработанной методики
- Взаимодействие с мобильным устройством с помощью мобильного приложения Smart Connect посредством технологии BLE

## Механизмы защиты
- Ограничение на количество подключений
- Ограничение физического взаимодействия
- Шифрование данных и прошивки
- Генераторы случайных числовых последовательностей
- Ограничение доступа к критическим областям памяти
- Использование новейших спецификаций BLE
- Защищенное подключение
- Динамический пин-код для аутентификации
- Распределение ключей шифрования при сопряжении

## Схема подключения электронных компонентов
![Пульсометр](https://github.com/MaxiEnergy/SmartPulse/assets/58640177/2f51a0b6-504c-4c47-aa35-b539a4d55911)

## Выпускная квалификационная работа
С оригиналом выпускной квалификационной работы можно ознакомиться по [ссылке](https://www.dropbox.com/scl/fi/evywal0odwmc1053ck2il/_.pdf?rlkey=i7d2i8soxyn0wn4oav7f0py9n&dl=0)

## Дополнительные материалы
Читайте про создание данного устройства в статье на Хабре
