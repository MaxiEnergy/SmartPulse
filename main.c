#include <stdio.h> 
#include <stdlib.h> 
#include <string.h> 
#include "freertos/FreeRTOS.h" // Подключение основных компонентов FreeRTOS
#include "freertos/task.h" // Подключение функционала для работы с задачами в FreeRTOS
#include "esp_log.h" // Подключение функций для логирования в ESP-IDF
#include "ssd1306.h" // Подключение библиотеки для работы с OLED-дисплеем на контроллере SSD1306
#include "font8x8_basic.h" // Подключение файла с базовым шрифтом для использования на дисплее
#include "driver/i2c.h" // Подключение драйвера I2C для ESP32
#include "driver/adc.h" // Подключение драйвера аналого-цифрового преобразователя
#include "driver/gpio.h" // Подключение драйвера для работы с GPIO
#include "esp_adc_cal.h" // Подключение библиотеки для калибровки ADC в ESP32
#include "freertos/event_groups.h" // Подключение функционала для работы с группами событий
#include "esp_event.h" // Подключение компонентов для работы с событиями в ESP-IDF
#include "nvs_flash.h" // Подключение функционала для работы с NVS (Non-Volatile Storage) в ESP32
#include "esp_nimble_hci.h" // Подключение библиотеки для HCI в стеке Bluetooth NimBLE
#include "nimble/nimble_port.h" // Основные компоненты порта NimBLE (Bluetooth stack)
#include "nimble/nimble_port_freertos.h" // Адаптация порта NimBLE для работы с FreeRTOS
#include "host/ble_hs.h" // Подключение функций высокого уровня для работы с Bluetooth в NimBLE
#include "services/gap/ble_svc_gap.h" // Подключение сервисов GAP для Bluetooth
#include "services/gatt/ble_svc_gatt.h" // Подключение сервисов GATT для Bluetooth
#include "sdkconfig.h" // Подключение файла конфигурации SDK (Software Development Kit)
#include "esp_random.h" // Подключение функции для генерации случайных чисел в ESP32

#define tag "SSD1306" // Определение тега для логирования, связанного с дисплеем SSD1306
#define I2C_MASTER_SCL_IO 9 // Назначение GPIO 9 для SCL (Serial Clock) линии I2C интерфейса
#define I2C_MASTER_SDA_IO 8 // Назначение GPIO 8 для SDA (Serial Data) линии I2C интерфейса
#define I2C_MASTER_FREQ_HZ 100000 // Установка скорости передачи данных I2C интерфейса
#define I2C_MASTER_TX_BUF_DISABLE 0 // Отключение буфера передачи для I2C мастера
#define I2C_MASTER_RX_BUF_DISABLE 0 // Отключение буфера приема для I2C мастера 
#define WRITE_BIT I2C_MASTER_WRITE // Определение бита записи для операций I2C
#define READ_BIT I2C_MASTER_READ // Определение бита чтения для операций I2C
#define ACK_CHECK_EN 0x1 // Включение проверки подтверждения (ACK) при взаимодействии по I2C
#define INA219_ADDR 0x40 // Адрес устройства INA219 на шине I2C

static const char *TAG = "main"; // Определение тега для логирования
static esp_adc_cal_characteristics_t adc1_chars; // Хранение характеристик АЦП канала 1 (ADC1) 
static char passkey[7]; // Статический массив для хранения пин-кода для аутентификации

uint16_t pulse_val_handle; // Переменная для хранения дескриптора значения пульса
uint16_t bat_val_handle; // Переменная для хранения дескриптора значения заряда батареи
uint16_t conn_handle; // Переменная для хранения дескриптора подключения BLE

static bool is_authenticated = false; // Статическая переменная для отслеживания состояния аутентификации пользователя
static uint16_t auth_handle; // Статическая переменная для хранения дескриптора аутентификации

uint8_t ble_addr_type; // Переменная для хранения типа адреса Bluetooth устройства
void ble_app_advertise(void); // Объявление функции, которая инициирует рассылку BLE
int Pulse;  // Переменная для хранения значения пульса
int Bat; // Переменная для хранения уровня заряда батареи

void generate_random_passkey() 
{
    uint32_t random_num = (esp_random() % 900000) + 100000; // Генерация случайного числа 
    snprintf(passkey, sizeof(passkey), "%06lu", random_num); // Форматирование числа в строку
    ESP_LOGI(TAG, "Generated passkey: %s", passkey); // Логирование сгенерированного кода
}

void i2c_master_init() 
{
    int i2c_master_port = I2C_NUM_0; // Назначение порта I2C
    i2c_config_t conf = { // Создание и конфигурация структуры для настроек I2C
        .mode = I2C_MODE_MASTER, // Установка режима работы на "мастер"
        .sda_io_num = I2C_MASTER_SDA_IO, // Установка GPIO для SDA линии
        .sda_pullup_en = GPIO_PULLUP_ENABLE, // Включение подтягивающего резистора для SDA
        .scl_io_num = I2C_MASTER_SCL_IO, // Установка GPIO для SCL линии
        .scl_pullup_en = GPIO_PULLUP_ENABLE, // Включение подтягивающего резистора для SCL
        .master.clk_speed = I2C_MASTER_FREQ_HZ, // Установка скорости тактового сигнала
    };
    i2c_param_config(i2c_master_port, &conf); // Конфигурация I2C с заданными параметрами
    i2c_driver_install(i2c_master_port, conf.mode, // Установка драйвера I2C
                       I2C_MASTER_RX_BUF_DISABLE, 
                       I2C_MASTER_TX_BUF_DISABLE, 0);
};

float read_battery_voltage() 
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create(); // Создание командной ссылки для I2C
    uint8_t data[2]; // Массив для хранения прочитанных данных
    i2c_master_start(cmd); // Инициирование стартового условия на шине I2C
    // Адрес с битом записи
    i2c_master_write_byte(cmd, (INA219_ADDR << 1) | WRITE_BIT, ACK_CHECK_EN); 
    i2c_master_write_byte(cmd, 0x02, ACK_CHECK_EN); // Запись в регистр устройства (0x02)
    i2c_master_start(cmd); // Повторное инициирование стартового условия
    i2c_master_write_byte(cmd, (INA219_ADDR << 1) | READ_BIT, ACK_CHECK_EN);
    // Отправка адреса устройства с битом чтения 
    i2c_master_read(cmd, data, 2, I2C_MASTER_LAST_NACK); // Чтение 2 байтов данных
    i2c_master_stop(cmd);  // Инициирование условия остановки на шине I2C
    // Отправка команд и ожидание завершения
    esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_PERIOD_MS); 
    i2c_cmd_link_delete(cmd); // Удаление командной ссылки I2C

    if (ret == ESP_OK) {
        // Преобразование двух байтов данных в число
        uint16_t voltage_raw = (data[0] << 8) | data[1]; 
        float voltage = voltage_raw >> 3; // Смещение вправо на 3 бита
        voltage *= 0.004; // Преобразование значения в напряжение (вольты)
        return voltage; // Возвращение измеренного напряжения
    } else {
	 // Логирование ошибки
        ESP_LOGE(TAG, "I2C communication failed: %s", esp_err_to_name(ret)); 
        return -1.0; // Возврат ошибочного значения в случае неудачи
    }
}

float calculate_battery_level(float voltage) 
{
    float max_voltage = 3.7; // Максимальное напряжение батареи (в вольтах)
    float min_voltage = 2.8; // Минимальное напряжение батареи (в вольтах)

    if (voltage >= max_voltage) return 100; // Батарея полностью заряжена
    if (voltage <= min_voltage) return 0; // Батарея полностью разряжена
    // Расчёт процента заряда батареи
    return ((voltage - min_voltage) / (max_voltage - min_voltage)) * 100; 
}

static int device_write_pincode(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg) 
{
    size_t om_len = OS_MBUF_PKTLEN(ctxt->om); // Получение длины пакета из контекста GATT
    char pincode[7]; // Массив для хранения пин-кода
    // Преобразование данных из mbuf в массив
    int rc = ble_hs_mbuf_to_flat(ctxt->om, pincode, sizeof(pincode) - 1, &om_len); 
    if (rc != 0) {
        return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN; // Возврат ошибки
    }
    pincode[om_len] = '\0'; // Добавление символа конца строки к пин-коду

    if (strcmp(pincode, passkey) == 0) { // Сравнение полученного пин-кода с сохраненным
        is_authenticated = true; // Установка флага аутентификации в случае успеха
	 // Логирование успешной аутентификации
        ESP_LOGI(TAG, "Authenticated with pincode: %s", pincode); 
    } else {
        is_authenticated = false; // Сброс флага аутентификации в случае несовпадения
        ESP_LOGE(TAG, "Invalid pincode: %s", pincode); // Логирование ошибки
    }

    return 0; // Возврат 0 в случае успешной обработки функции
}

static int device_read_Pulse(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    if (!is_authenticated) { // Проверка состояния аутентификации
        return BLE_ATT_ERR_INSUFFICIENT_AUTHEN; // Возвращение ошибки
    } else {
	 // Добавление значения пульса в буфер передачи, если аутентифицирован
        os_mbuf_append(ctxt->om, &Pulse, sizeof(int)); 
        return 0; // Возвращение 0, указывая на успешное выполнение
    }
}

static int device_read_Bat(uint16_t con_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    if (!is_authenticated) { // Проверка, прошел ли пользователь аутентификацию
        return BLE_ATT_ERR_INSUFFICIENT_AUTHEN; // Возвращение ошибки
    }
    else {
	 // Добавление значения заряда батареи в буфер передачи
        os_mbuf_append(ctxt->om, &Bat, sizeof(int)); 
        return 0; // Возвращение 0, указывая на успешное выполнение
    }
}

static const struct ble_gatt_svc_def gatt_svcs[] = 
{
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY, // Тип сервиса: основной
        .uuid = BLE_UUID16_DECLARE(0x180), // UUID сервиса
        .characteristics = (struct ble_gatt_chr_def[]){ // Массив характеристик сервиса
            {
                .uuid = BLE_UUID16_DECLARE(0xFEF3), // UUID характеристики для пин-кода
                .flags = BLE_GATT_CHR_F_WRITE, // Флаги характеристики: только запись
                .access_cb = device_write_pincode, // Функция обратного вызова
                .val_handle = &auth_handle, // Указатель на дескриптор значения
            },
            {
                .uuid = BLE_UUID16_DECLARE(0xFEF4), // UUID характеристики для пульса
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY, // Чтение и уведомление
                .access_cb = device_read_Pulse, // Функция обратного вызова для чтения пульса
                .val_handle = &pulse_val_handle, // Указатель на дескриптор значения пульса
            },
            {
                .uuid = BLE_UUID16_DECLARE(0xFEF5), // UUID характеристики для заряда батареи
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY, // Чтение и уведомление
                .access_cb = device_read_Bat, // Функция обратного вызова
                .val_handle = &bat_val_handle, // Указатель на дескриптор значения заряда батареи
            },
            {0}, // Обнуление массива характеристик
        },
    },
    {0}, // Обнуление массива сервисов
};

void ble_notify(uint16_t attr_handle, int value)
{
    if (!is_authenticated) { // Проверка, аутентифицирован ли пользователь
        return; // Выход из функции, если пользователь не аутентифицирован
    }
    else {
	 // Создание буфера сообщений из значения
        struct os_mbuf *om = ble_hs_mbuf_from_flat(&value, sizeof(value)); 
	 // Отправка уведомления с использованием кастомного значения
        int rc = ble_gattc_notify_custom(conn_handle, attr_handle, om); 
        if (rc) {
            ESP_LOGE(TAG, "Error sending notification; rc=%d", rc); // Логирование ошибки
        }
    }
}
static int ble_gap_event(struct ble_gap_event *event, void *arg)
{
    switch (event->type) // Определение типа события GAP
    {
    case BLE_GAP_EVENT_CONNECT: // Событие подключения
        ESP_LOGI("GAP", "BLE GAP EVENT CONNECT %s", event->connect.status == 0 ? "OK!" : "FAILED!"); // Логирование результата подключения
        if (event->connect.status != 0) // Если подключение не удалось
        {
            ble_app_advertise();// Повторный запуск рассылки
        }
        conn_handle = event->connect.conn_handle; // Сохранение дескриптора подключения
        break;
    case BLE_GAP_EVENT_DISCONNECT: // Событие отключения
      conn_handle = 0; // Сброс дескриптора подключения
    break;
    case BLE_GAP_EVENT_PASSKEY_ACTION: { // Событие действия с пин-кодом
        struct ble_sm_io pkey_io = {0}; // Структура для ввода/вывода пин-кода
        switch(event->passkey.params.action) { // Определение действия с пин-кодом
    case BLE_SM_IOACT_INPUT:
      break;
    case BLE_SM_IOACT_DISP:
      pkey_io.action = BLE_SM_IOACT_DISP; // Установка действия на отображение пин-кода
      pkey_io.passkey = passkey; // Установка пин-кода для отображения
        break;
        }
	 // Инъекция ввода/вывода пин-кода
        int rc = ble_sm_inject_io(event->passkey.conn_handle, &pkey_io); 
        assert(rc == 0); // Проверка успешности операции
        break;
    }
    case BLE_GAP_EVENT_ADV_COMPLETE: // Событие завершения рассылки
        ESP_LOGI("GAP", "BLE GAP EVENT"); // Логирование события
        ble_app_advertise(); // Запуск рассылки
        break;
    default:
        break; // Для всех остальных событий ничего не делаем
    }
    return 0; // Возвращаем 0, указывая на успешное выполнение
}

void ble_app_advertise(void)
{
    struct ble_hs_adv_fields fields; // Структура для настройки полей сообщения рассылки
    const char *device_name; // Указатель на имя устройства
    memset(&fields, 0, sizeof(fields)); // Инициализация структуры fields нулями
    device_name = ble_svc_gap_device_name(); // Получение имени устройства
    fields.name = (uint8_t *)device_name; // Установка имени устройства в поля рассылки
    fields.name_len = strlen(device_name); // Установка длины имени устройства
    fields.name_is_complete = 1; // Указание на то, что имя устройства полное
    ble_gap_adv_set_fields(&fields); // Установка настроенных полей в сообщении рассылки

    struct ble_gap_adv_params adv_params; // Структура для настройки параметров рассылки
    memset(&adv_params, 0, sizeof(adv_params)); // Инициализация структуры adv_params нулями
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND; // Режим подключения: непрерывный (undirected)
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN; // Режим обнаружения: общий (general)
    // Запуск рассылки
    ble_gap_adv_start(ble_addr_type, NULL, BLE_HS_FOREVER, &adv_params, ble_gap_event, NULL); 
}

void ble_app_on_sync(void)
{
    ble_hs_id_infer_auto(0, &ble_addr_type); // Автоматическое определение типа адреса
    ble_app_advertise(); // Запуск процесса рассылки BLE
    ble_hs_cfg.sm_io_cap = BLE_HS_IO_DISPLAY_ONLY; // Установка возможностей: только отображение
    ble_hs_cfg.sm_bonding = 1; // Включение процесса связывания (bonding) для BLE
    ble_hs_cfg.sm_mitm = 1; // Включение защиты от "Man-In-The-Middle" (человек посередине) атак
    ble_hs_cfg.sm_sc = 1; // Включение защиты с использованием Secure Connections (SC)
    ble_hs_cfg.sm_our_key_dist = 1; // Установка распределения ключей для устройства
    ble_hs_cfg.sm_their_key_dist = 1; // Распределение ключей для подключаемого устройства
}
void host_task(void *param)
{
    nimble_port_run(); // функция будет выполняться, пока не будет вызвана nimble_port_stop()
}

void app_main(void)
{
  generate_random_passkey();// Генерация случайного пин-кода для аутентификации
  int rc; // Переменная для хранения кода результата операций
  nvs_flash_init(); // Инициализация NVS для хранения данных
  nimble_port_init(); // Инициализация порта NimBLE 
  ble_svc_gap_device_name_set("SmartPulse"); // Установка имени устройства
  ble_svc_gap_init(); // Инициализация сервиса GAP для Bluetooth
  ble_svc_gatt_init(); // Инициализация сервиса GATT для Bluetooth
  rc = ble_gatts_count_cfg(gatt_svcs); // Подсчет конфигураций GATT сервисов
  assert(rc == 0); // Проверка успешности операции подсчета
  rc = ble_gatts_add_svcs(gatt_svcs); // Добавление GATT сервисов
  assert(rc == 0); // Проверка успешности операции добавления
  ble_hs_cfg.sync_cb = ble_app_on_sync; // Установка функции обратного вызова
  nimble_port_freertos_init(host_task); // Инициализация NimBLE с использованием FreeRTOS  
    
  gpio_set_direction(GPIO_NUM_10, GPIO_MODE_INPUT); // Установка GPIO 10 как входного

  SSD1306_t dev; // Объявление переменной для драйвера OLED дисплея SSD1306

  #if CONFIG_SPI_INTERFACE // Проверка, включен ли SPI интерфейс в конфигурации
  ESP_LOGI(tag, "INTERFACE is SPI"); // Логирование использования SPI интерфейса
  ESP_LOGI(tag, "CONFIG_MOSI_GPIO=%d",CONFIG_MOSI_GPIO); // Логирование номера GPIO для MOSI
  ESP_LOGI(tag, "CONFIG_SCLK_GPIO=%d",CONFIG_SCLK_GPIO); // Логирование номера GPIO для SCLK
  ESP_LOGI(tag, "CONFIG_CS_GPIO=%d",CONFIG_CS_GPIO); // Логирование номера GPIO для CS
  ESP_LOGI(tag, "CONFIG_DC_GPIO=%d",CONFIG_DC_GPIO); // Логирование номера GPIO для DC
  ESP_LOGI(tag, "CONFIG_RESET_GPIO=%d",CONFIG_RESET_GPIO); // Логирование номера GPIO для RESET
  spi_master_init(&dev, CONFIG_MOSI_GPIO, CONFIG_SCLK_GPIO, // Инициализация SPI интерфейса
                CONFIG_CS_GPIO, CONFIG_DC_GPIO, CONFIG_RESET_GPIO);
  #endif 

  #if CONFIG_SSD1306_128x64 // Проверка, выбран ли OLED дисплей размером 128x64
  ESP_LOGI(tag, "Panel is 128x64"); // Логирование использования 128x64
  ssd1306_init(&dev, 128, 64); // Инициализация дисплея SSD1306 с разрешением 128x64
  #endif 

  i2c_master_init(); // Инициализация I2C интерфейса в роли мастера

  esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, // Калибровка АЦП
                         ADC_WIDTH_BIT_DEFAULT, 0, &adc1_chars);

  adc1_config_width(ADC_WIDTH_BIT_DEFAULT); // Настройка ширины АЦП
  adc1_config_channel_atten(ADC1_CHANNEL_3, ADC_ATTEN_DB_11);    

  ssd1306_clear_screen(&dev, false); // Очистка экрана OLED дисплея
  ssd1306_contrast(&dev, 0xff); // Установка контрастности экрана
  ssd1306_display_text(&dev, 0, "PIN", 3, false); // Отображение текста "PIN" на экране
  ssd1306_display_text(&dev, 2, passkey, 6, false); // Отображение сгенерированного пин-кода
  vTaskDelay(5000 / portTICK_PERIOD_MS); // Задержка на 5 секунд
  ssd1306_clear_screen(&dev, false); // Очистка экрана после задержки

  while (1)
  {
       int touch = gpio_get_level(GPIO_NUM_10); // Чтение уровня сигнала на GPIO 10 
       float voltage = read_battery_voltage(); // Чтение напряжения батареи
       char str1[20]; // Буфер для строки, представляющей уровень заряда батареи
       char str2[20]; // Буфер для строки, представляющей пульс

       int adc_value = adc1_get_raw(ADC1_CHANNEL_3); // Чтение значения с АЦП
         int BPM = (adc_value - 2360) / 100; // Расчет пульса на основе значения АЦП
         if (BPM > 0)
            Pulse = BPM + 60; // Корректировка пульса и сохранение в глобальной переменной
        else
            Pulse = 0; // Установка пульса в 0 при отрицательном расчетном значении

        if (voltage >= 0){
	     // Расчет уровня заряда батареи
            float battery_level = calculate_battery_level(voltage); 
            Bat = (int)battery_level; // Сохранение уровня заряда в глобальной переменной
        }
        else{
            Bat = 0; // Установка уровня заряда в 0 при отрицательном напряжении
        }

        sprintf(str1, "%d", Bat); // Форматирование уровня заряда батареи в строку
        sprintf(str2, "%d", Pulse); // Форматирование пульса в строку

        if (conn_handle != 0){ // Проверка на наличие активного BLE подключения
            ble_notify(pulse_val_handle, Pulse); // Отправка уведомления о пульсе
            ble_notify(bat_val_handle, Bat); // Отправка уведомления об уровне заряда батареи
        }

    if (touch == 1){ // Если нажали на датчик касания
          ssd1306_clear_screen(&dev, false); // Очистка экрана OLED
          ssd1306_contrast(&dev, 0xff); // Установка максимальной контрастности экрана
          ssd1306_display_text(&dev, 0, "Battery", 7, false); // Отображение текста "Battery"
          ssd1306_display_text(&dev, 2, str1, 3, false); // Отображение уровня заряда батареи
          ssd1306_display_text(&dev, 4, "Pulse", 5, false); // Отображение текста "Pulse"
          ssd1306_display_text(&dev, 6, str2, 3, false); // Отображение измеренного пульса
          vTaskDelay(1000 / portTICK_PERIOD_MS); // Задержка на 1 секунду
    } else {
          ssd1306_clear_screen(&dev, false); // Очистка экрана OLED, если датчик касания выключен
    }
    }
} 
