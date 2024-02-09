#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "ssd1306.h"
#include "font8x8_basic.h"
#include "driver/i2c.h"
#include "driver/adc.h"
#include "driver/gpio.h"
#include "esp_adc_cal.h"
#include "freertos/event_groups.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "esp_nimble_hci.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"
#include "sdkconfig.h"
#include "esp_random.h"

#define tag "SSD1306"
#define I2C_MASTER_SCL_IO 9  /*!< gpio number for I2C master clock */
#define I2C_MASTER_SDA_IO 8  /*!< gpio number for I2C master data  */
#define I2C_MASTER_FREQ_HZ 100000 /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE 0 /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 0 /*!< I2C master doesn't need buffer */
#define WRITE_BIT I2C_MASTER_WRITE /*!< I2C master write */
#define READ_BIT I2C_MASTER_READ /*!< I2C master read */
#define ACK_CHECK_EN 0x1 /*!< I2C master will check ack from slave*/
#define INA219_ADDR 0x40 /*!< slave address for INA219 sensor */

static const char *TAG = "main";
static esp_adc_cal_characteristics_t adc1_chars;
static char passkey[7];

uint16_t pulse_val_handle;
uint16_t bat_val_handle;
uint16_t conn_handle;

static bool is_authenticated = false;
static uint16_t auth_handle;

uint8_t ble_addr_type;
void ble_app_advertise(void);
int Pulse;
int Bat;

void generate_random_passkey() {
    // Генерируем случайное число от 0 до 899999, затем добавляем 100000
    uint32_t random_num = (esp_random() % 900000) + 100000; // Диапазон теперь от 100000 до 999999
    snprintf(passkey, sizeof(passkey), "%06lu", random_num); // Форматируем как строку
    ESP_LOGI(TAG, "Generated passkey: %s", passkey);
}

void i2c_master_init() {
    int i2c_master_port = I2C_NUM_0;
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    i2c_param_config(i2c_master_port, &conf);
    i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
};

float read_battery_voltage() {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    uint8_t data[2];
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (INA219_ADDR << 1) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, 0x02, ACK_CHECK_EN);  // Pointer to "Bus Voltage" register
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (INA219_ADDR << 1) | READ_BIT, ACK_CHECK_EN);
    i2c_master_read(cmd, data, 2, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    if (ret == ESP_OK) {
        uint16_t voltage_raw = (data[0] << 8) | data[1];
        float voltage = voltage_raw >> 3;
        voltage *= 0.004;  // Convert to volts
        return voltage;
    } else {
        ESP_LOGE(TAG, "I2C communication failed: %s", esp_err_to_name(ret));
        return -1.0;
    }
}

float calculate_battery_level(float voltage) {
    // Assuming a 3.7V LiPo battery
    // You can adjust these values based on your specific battery's discharge curve
    float max_voltage = 3.7;
    float min_voltage = 2.8;

    if (voltage >= max_voltage) return 100;
    if (voltage <= min_voltage) return 0;

    return ((voltage - min_voltage) / (max_voltage - min_voltage)) * 100;
}


// Функция записи пин-кода в характеристику
static int device_write_pincode(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg) {
    size_t om_len = OS_MBUF_PKTLEN(ctxt->om);
    char pincode[7]; // Максимальная длина пин-кода + 1 для нуль-терминатора
    int rc = ble_hs_mbuf_to_flat(ctxt->om, pincode, sizeof(pincode) - 1, &om_len);
    if (rc != 0) {
        return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
    }
    pincode[om_len] = '\0'; // Установить нуль-терминатор

    if (strcmp(pincode, passkey) == 0) {
        is_authenticated = true;
        ESP_LOGI(TAG, "Authenticated with pincode: %s", pincode);
    } else {
        is_authenticated = false;
        ESP_LOGE(TAG, "Invalid pincode: %s", pincode);
    }

    return 0;
}

// Функция чтения характеристики
static int device_read_Pulse(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg)
{
	if (!is_authenticated){
		return BLE_ATT_ERR_INSUFFICIENT_AUTHEN;
	} else{
    os_mbuf_append(ctxt->om, &Pulse, sizeof(int));
    return 0;}
}

static int device_read_Bat(uint16_t con_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg)
{
	if (!is_authenticated){
		return BLE_ATT_ERR_INSUFFICIENT_AUTHEN;
	}
	else {
    os_mbuf_append(ctxt->om, &Bat, sizeof(int));
    return 0;}
}

// Определение сервиса и характеристики
static const struct ble_gatt_svc_def gatt_svcs[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = BLE_UUID16_DECLARE(0x180), // UUID для устройства
        .characteristics = (struct ble_gatt_chr_def[]){
            {
                .uuid = BLE_UUID16_DECLARE(0xFEF3), // UUID для чтения и уведомлений
                .flags = BLE_GATT_CHR_F_WRITE,
                .access_cb = device_write_pincode,
				.val_handle = &auth_handle,
            },
            {
                .uuid = BLE_UUID16_DECLARE(0xFEF4), // UUID для чтения и уведомлений
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY,
                .access_cb = device_read_Pulse,
				.val_handle = &pulse_val_handle,
            },
            {
                .uuid = BLE_UUID16_DECLARE(0xFEF5), // UUID для чтения
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY,
                .access_cb = device_read_Bat,
				.val_handle = &bat_val_handle,
            },
            {0},
        },
    },
    {0},
};

void ble_notify(uint16_t attr_handle, int value){
	if(!is_authenticated){
		return;
	}
	else{
	struct os_mbuf *om = ble_hs_mbuf_from_flat(&value, sizeof(value));
	int rc = ble_gattc_notify_custom(conn_handle, attr_handle, om);
	if (rc) {
		ESP_LOGE(TAG, "Error sending notification; rc=%d", rc);
	}
	}
}

// BLE event handling
static int ble_gap_event(struct ble_gap_event *event, void *arg)
{
    switch (event->type)
    {
    // Advertise if connected
    case BLE_GAP_EVENT_CONNECT:
        ESP_LOGI("GAP", "BLE GAP EVENT CONNECT %s", event->connect.status == 0 ? "OK!" : "FAILED!");
        if (event->connect.status != 0)
        {
            ble_app_advertise();
        }
        conn_handle = event->connect.conn_handle;
        break;
    case BLE_GAP_EVENT_DISCONNECT:
    	conn_handle = 0;
    break;
    case BLE_GAP_EVENT_PASSKEY_ACTION: {
        struct ble_sm_io pkey_io = {0};
        switch(event->passkey.params.action) {
            case BLE_SM_IOACT_INPUT:
                // Требуется ввести пин-код на устройстве
                // Здесь должен быть код для ввода пин-кода пользователем
                break;
            case BLE_SM_IOACT_DISP:
                // Отобразите пин-код на устройстве
                pkey_io.action = BLE_SM_IOACT_DISP;
                pkey_io.passkey = passkey;
                // Здесь должен быть код для отображения пин-кода пользователю
                break;
        }
        int rc = ble_sm_inject_io(event->passkey.conn_handle, &pkey_io);
        assert(rc == 0);
        break;
    }
    // Advertise again after completion of the event
    case BLE_GAP_EVENT_ADV_COMPLETE:
        ESP_LOGI("GAP", "BLE GAP EVENT");
        ble_app_advertise();
        break;
    default:
        break;
    }
    return 0;
}

// Define the BLE connection
void ble_app_advertise(void)
{
    // GAP - device name definition
    struct ble_hs_adv_fields fields;
    const char *device_name;
    memset(&fields, 0, sizeof(fields));
    device_name = ble_svc_gap_device_name(); // Read the BLE device name
    fields.name = (uint8_t *)device_name;
    fields.name_len = strlen(device_name);
    fields.name_is_complete = 1;
    ble_gap_adv_set_fields(&fields);

    // GAP - device connectivity definition
    struct ble_gap_adv_params adv_params;
    memset(&adv_params, 0, sizeof(adv_params));
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND; // connectable or non-connectable
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN; // discoverable or non-discoverable
    ble_gap_adv_start(ble_addr_type, NULL, BLE_HS_FOREVER, &adv_params, ble_gap_event, NULL);
}

// The application
void ble_app_on_sync(void)
{
    ble_hs_id_infer_auto(0, &ble_addr_type); // Determines the best address type automatically
    ble_app_advertise();                     // Define the BLE connection
    // Используйте правильные определения для вашей версии ESP-IDF и NimBLE.
    // Например, для NimBLE, это может быть BLE_HS_IO_DISPLAY_ONLY.
    ble_hs_cfg.sm_io_cap = BLE_HS_IO_DISPLAY_ONLY;
    ble_hs_cfg.sm_bonding = 1;
    ble_hs_cfg.sm_mitm = 1; // Установите этот флаг, если требуется защита от MITM.
    ble_hs_cfg.sm_sc = 1;   // Используйте безопасное соединение (Secure Connection).
    ble_hs_cfg.sm_our_key_dist = 1;
    ble_hs_cfg.sm_their_key_dist = 1;
}

// The infinite task
void host_task(void *param)
{
    nimble_port_run(); // This function will return only when nimble_port_stop() is executed
}

void app_main(void)
{
	generate_random_passkey();
	int rc;
    nvs_flash_init();                          // 1 - Initialize NVS flash using
    //esp_nimble_hci_and_controller_init();      // 2 - Initialize ESP controller
    nimble_port_init();                        // 3 - Initialize the host stack
    ble_svc_gap_device_name_set("SmartPulse"); // 4 - Initialize NimBLE configuration - server name
    ble_svc_gap_init();                        // 4 - Initialize NimBLE configuration - gap service
    ble_svc_gatt_init();                       // 4 - Initialize NimBLE configuration - gatt service
    rc = ble_gatts_count_cfg(gatt_svcs);            // 4 - Initialize NimBLE configuration - config gatt services
    assert(rc==0);
    rc = ble_gatts_add_svcs(gatt_svcs);             // 4 - Initialize NimBLE configuration - queues gatt services.
    assert(rc==0);
    ble_hs_cfg.sync_cb = ble_app_on_sync;      // 5 - Initialize application
    nimble_port_freertos_init(host_task);      // 6 - Run the thread
    
gpio_set_direction(GPIO_NUM_10, GPIO_MODE_INPUT);
 SSD1306_t dev;

 #if CONFIG_SPI_INTERFACE
 ESP_LOGI(tag, "INTERFACE is SPI");
 ESP_LOGI(tag, "CONFIG_MOSI_GPIO=%d",CONFIG_MOSI_GPIO);
 ESP_LOGI(tag, "CONFIG_SCLK_GPIO=%d",CONFIG_SCLK_GPIO);
 ESP_LOGI(tag, "CONFIG_CS_GPIO=%d",CONFIG_CS_GPIO);
 ESP_LOGI(tag, "CONFIG_DC_GPIO=%d",CONFIG_DC_GPIO);
 ESP_LOGI(tag, "CONFIG_RESET_GPIO=%d",CONFIG_RESET_GPIO);
 spi_master_init(&dev, CONFIG_MOSI_GPIO, CONFIG_SCLK_GPIO, CONFIG_CS_GPIO, CONFIG_DC_GPIO, CONFIG_RESET_GPIO);
 #endif // CONFIG_SPI_INTERFACE

 #if CONFIG_SSD1306_128x64
 ESP_LOGI(tag, "Panel is 128x64");
 ssd1306_init(&dev, 128, 64);
 #endif // CONFIG_SSD1306_128x64
 
    i2c_master_init();

    esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_DEFAULT, 0, &adc1_chars);

    adc1_config_width(ADC_WIDTH_BIT_DEFAULT);
    adc1_config_channel_atten(ADC1_CHANNEL_3, ADC_ATTEN_DB_11);

    ssd1306_clear_screen(&dev, false);
    ssd1306_contrast(&dev, 0xff);
    ssd1306_display_text(&dev, 0, "PIN", 3, false);
    ssd1306_display_text(&dev, 2, passkey, 6, false);
    vTaskDelay(5000 / portTICK_PERIOD_MS);
	ssd1306_clear_screen(&dev, false);
	
    while (1)
    {
    int touch = gpio_get_level(GPIO_NUM_10);
     float voltage = read_battery_voltage();
     char str1[20];
     char str2[20];
        int adc_value = adc1_get_raw(ADC1_CHANNEL_3);
        int BPM = (adc_value - 2360) / 100;
                if (BPM > 0)
                 Pulse = BPM + 60;
                else
                 Pulse = 0;
                if (voltage >= 0){
                    float battery_level = calculate_battery_level(voltage);
                 Bat = (int)battery_level;}
                else{
                    Bat = 0;}
                sprintf(str1, "%d", Bat);
                sprintf(str2, "%d", Pulse);
                if (conn_handle != 0){
                	ble_notify(pulse_val_handle, Pulse);
                	ble_notify(bat_val_handle, Bat);
                }
                // Где-то в вашем коде, например, в задаче FreeRTOS или обработчике событий:
                // Инициализируйте параметры уведомлений и вызовите функцию send_notification при каждом обновлении данных.
                //init_notify_params(conn_handle, gatt_chr_defs[0].val_handle, &Pulse, sizeof(int));
                //send_notification();
              if (touch == 1){
             ssd1306_clear_screen(&dev, false);
             ssd1306_contrast(&dev, 0xff);
             ssd1306_display_text(&dev, 0, "Battery", 7, false);
             ssd1306_display_text(&dev, 2, str1, 3, false);
             ssd1306_display_text(&dev, 4, "Pulse", 5, false);
             ssd1306_display_text(&dev, 6, str2, 3, false);
                     //printf("Touch: %d", touch);
                     //printf("\n");
             vTaskDelay(1000 / portTICK_PERIOD_MS);}
              else
            	  ssd1306_clear_screen(&dev, false);
                //printf("Battery Level: %d", Bat);
                //printf("\n");
                //printf("Pulse: %d", Pulse);
                //printf("\n");
            }

        }
