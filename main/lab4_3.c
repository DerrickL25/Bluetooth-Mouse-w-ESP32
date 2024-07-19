/*
 * SPDX-FileCopyrightText: 2021-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_bt.h"

#include "esp_hidd_prf_api.h"
#include "esp_bt_defs.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_gatt_defs.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "driver/gpio.h"
#include "hid_dev.h"

////////////////////////////////////////////////// lab4_1 stuff
#include <stdio.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"

// I2C config constants
#define I2C_MASTER_SCL_IO    8              // GPIO number for I2C master clock
#define I2C_MASTER_SDA_IO    10             // GPIO number for I2C master data
#define I2C_MASTER_NUM       I2C_NUM_0      // I2C port number for master dev
#define I2C_MASTER_FREQ_HZ  400000          // I2C master clock frequency
#define I2C_MASTER_TX_BUF_DISABLE   0       // I2C master doesn't need buffer
#define I2C_MASTER_RX_BUF_DISABLE   0       // I2C master doesn't need buffer

// IMU address and registers
#define ICM42670_ADDR        0x68           // ICM-42670-P I2C address

static const char* TAG = "lab4_1";

void i2c_master_init() {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    i2c_param_config(I2C_MASTER_NUM, &conf);
    i2c_driver_install(I2C_MASTER_NUM, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}

void write_register(uint8_t reg_addr, uint8_t data) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (ICM42670_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);
    i2c_master_write_byte(cmd, data, true);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
}

esp_err_t i2c_master_read_sensor_data(uint8_t reg_addr, uint8_t *data_rd, size_t size) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (ICM42670_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (ICM42670_ADDR << 1) | I2C_MASTER_READ, true);
    if (size > 1) {
        i2c_master_read(cmd, data_rd, size - 1, I2C_MASTER_ACK);
    }
    i2c_master_read_byte(cmd, data_rd + size - 1, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    return ret;
}

void imu_task(void *pvParameters) {
    uint8_t data[6];
    while (1) {
        if(i2c_master_read_sensor_data(0x0B, data, 6) == ESP_OK) { // Read 6 bytes starting from X accelerometer data register (0x0B)
            int16_t x = (data[0] << 8) | data[1]; // Combine high and low bytes for X axis
            int16_t y = (data[2] << 8) | data[3]; // Combine high and low bytes for Y axis
            //int16_t z = (data[4] << 8) | data[5]; // Combine high and low bytes for Z axis
            //printf("X: %d, Y: %d, Z: %d\n", x, y, z);

            // printing board orientation
            int delt = 250;
            if((x > delt) && ((y < delt) && (y > -delt))) {    // tilted left
                ESP_LOGI(TAG, "LEFT");
            } else if ((x > delt) && (y < -delt)) {
                ESP_LOGI(TAG, "DOWN LEFT");
            } else if ((x > delt) && (y > delt)) {
                ESP_LOGI(TAG, "UP LEFT");
            } else if ((x < -delt) && ((y < delt) && (y > -delt))) {    // tilted right
                ESP_LOGI(TAG, "RIGHT");
            } else if ((x < -delt) && (y < -delt)) {
                ESP_LOGI(TAG, "DOWN RIGHT");
            } else if ((x < -delt) && (y > delt)) {
                ESP_LOGI(TAG, "UP RIGHT");
            } else if ((y < -delt) && ((x < delt) && (x > -delt))) {     // tilted down
                ESP_LOGI(TAG, "DOWN");
            } else if ((y > delt) && ((x < delt) && (x > -delt))) {     // tilted up
                ESP_LOGI(TAG, "UP");
            } else {
                ESP_LOGI(TAG, "FLAT");
            }

        } else {printf("error reading sensor data\n");}
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
////////////////////////////////////////////////////////////////

/**
 * Brief:
 * This example Implemented BLE HID device profile related functions, in which the HID device
 * has 4 Reports (1 is mouse, 2 is keyboard and LED, 3 is Consumer Devices, 4 is Vendor devices).
 * Users can choose different reports according to their own application scenarios.
 * BLE HID profile inheritance and USB HID class.
 */

/**
 * Note:
 * 1. Win10 does not support vendor report , So SUPPORT_REPORT_VENDOR is always set to FALSE, it defines in hidd_le_prf_int.h
 * 2. Update connection parameters are not allowed during iPhone HID encryption, slave turns
 * off the ability to automatically update connection parameters during encryption.
 * 3. After our HID device is connected, the iPhones write 1 to the Report Characteristic Configuration Descriptor,
 * even if the HID encryption is not completed. This should actually be written 1 after the HID encryption is completed.
 * we modify the permissions of the Report Characteristic Configuration Descriptor to `ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE_ENCRYPTED`.
 * if you got `GATT_INSUF_ENCRYPTION` error, please ignore.
 */

#define HID_DEMO_TAG "HID_DEMO"


static uint16_t hid_conn_id = 0;
static bool sec_conn = false;
#define CHAR_DECLARATION_SIZE   (sizeof(uint8_t))

static void hidd_event_callback(esp_hidd_cb_event_t event, esp_hidd_cb_param_t *param);

#define HIDD_DEVICE_NAME            "HID"
static uint8_t hidd_service_uuid128[] = {
    /* LSB <--------------------------------------------------------------------------------> MSB */
    //first uuid, 16bit, [12],[13] is the value
    0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0x12, 0x18, 0x00, 0x00,
};

static esp_ble_adv_data_t hidd_adv_data = {
    .set_scan_rsp = false,
    .include_name = true,
    .include_txpower = true,
    .min_interval = 0x0006, //slave connection min interval, Time = min_interval * 1.25 msec
    .max_interval = 0x0010, //slave connection max interval, Time = max_interval * 1.25 msec
    .appearance = 0x03c0,       //HID Generic,
    .manufacturer_len = 0,
    .p_manufacturer_data =  NULL,
    .service_data_len = 0,
    .p_service_data = NULL,
    .service_uuid_len = sizeof(hidd_service_uuid128),
    .p_service_uuid = hidd_service_uuid128,
    .flag = 0x6,
};

static esp_ble_adv_params_t hidd_adv_params = {
    .adv_int_min        = 0x20,
    .adv_int_max        = 0x30,
    .adv_type           = ADV_TYPE_IND,
    .own_addr_type      = BLE_ADDR_TYPE_PUBLIC,
    //.peer_addr            =
    //.peer_addr_type       =
    .channel_map        = ADV_CHNL_ALL,
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};


static void hidd_event_callback(esp_hidd_cb_event_t event, esp_hidd_cb_param_t *param)
{
    switch(event) {
        case ESP_HIDD_EVENT_REG_FINISH: {
            if (param->init_finish.state == ESP_HIDD_INIT_OK) {
                //esp_bd_addr_t rand_addr = {0x04,0x11,0x11,0x11,0x11,0x05};
                esp_bt_dev_set_device_name(HIDD_DEVICE_NAME);
                esp_ble_gap_config_adv_data(&hidd_adv_data);

            }
            break;
        }
        case ESP_BAT_EVENT_REG: {
            break;
        }
        case ESP_HIDD_EVENT_DEINIT_FINISH:
	     break;
		case ESP_HIDD_EVENT_BLE_CONNECT: {
            ESP_LOGI(HID_DEMO_TAG, "ESP_HIDD_EVENT_BLE_CONNECT");
            hid_conn_id = param->connect.conn_id;
            break;
        }
        case ESP_HIDD_EVENT_BLE_DISCONNECT: {
            sec_conn = false;
            ESP_LOGI(HID_DEMO_TAG, "ESP_HIDD_EVENT_BLE_DISCONNECT");
            esp_ble_gap_start_advertising(&hidd_adv_params);
            break;
        }
        case ESP_HIDD_EVENT_BLE_VENDOR_REPORT_WRITE_EVT: {
            ESP_LOGI(HID_DEMO_TAG, "%s, ESP_HIDD_EVENT_BLE_VENDOR_REPORT_WRITE_EVT", __func__);
            ESP_LOG_BUFFER_HEX(HID_DEMO_TAG, param->vendor_write.data, param->vendor_write.length);
            break;
        }
        case ESP_HIDD_EVENT_BLE_LED_REPORT_WRITE_EVT: {
            ESP_LOGI(HID_DEMO_TAG, "ESP_HIDD_EVENT_BLE_LED_REPORT_WRITE_EVT");
            ESP_LOG_BUFFER_HEX(HID_DEMO_TAG, param->led_write.data, param->led_write.length);
            break;
        }
        default:
            break;
    }
    return;
}

static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    switch (event) {
    case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
        esp_ble_gap_start_advertising(&hidd_adv_params);
        break;
     case ESP_GAP_BLE_SEC_REQ_EVT:
        for(int i = 0; i < ESP_BD_ADDR_LEN; i++) {
             ESP_LOGD(HID_DEMO_TAG, "%x:",param->ble_security.ble_req.bd_addr[i]);
        }
        esp_ble_gap_security_rsp(param->ble_security.ble_req.bd_addr, true);
	 break;
     case ESP_GAP_BLE_AUTH_CMPL_EVT:
        sec_conn = true;
        esp_bd_addr_t bd_addr;
        memcpy(bd_addr, param->ble_security.auth_cmpl.bd_addr, sizeof(esp_bd_addr_t));
        ESP_LOGI(HID_DEMO_TAG, "remote BD_ADDR: %08x%04x",\
                (bd_addr[0] << 24) + (bd_addr[1] << 16) + (bd_addr[2] << 8) + bd_addr[3],
                (bd_addr[4] << 8) + bd_addr[5]);
        ESP_LOGI(HID_DEMO_TAG, "address type = %d", param->ble_security.auth_cmpl.addr_type);
        ESP_LOGI(HID_DEMO_TAG, "pair status = %s",param->ble_security.auth_cmpl.success ? "success" : "fail");
        if(!param->ble_security.auth_cmpl.success) {
            ESP_LOGE(HID_DEMO_TAG, "fail reason = 0x%x",param->ble_security.auth_cmpl.fail_reason);
        }
        break;
    default:
        break;
    }
}

void hid_demo_task(void *pvParameters)
{
    
    vTaskDelay(1000 / portTICK_PERIOD_MS); // Initial delay of 1 second
    uint8_t buttons_state = 0;             // No buttons pressed

    while (1) {
        if (sec_conn) {
            uint8_t data[6];
            while (1) {
                if(i2c_master_read_sensor_data(0x0B, data, 6) == ESP_OK) { // Read 6 bytes starting from X accelerometer data register (0x0B)
                    int16_t x = (data[0] << 8) | data[1]; // Combine high and low bytes for X axis
                    int16_t y = (data[2] << 8) | data[3]; // Combine high and low bytes for Y axis
                    //printf("X: %d, Y: %d\n", x, y);

                    int8_t x_movement = 3;                // Movement value for left-right
                    int8_t y_movement = 3;                // No vertical movement
                    int delt = 350;

                    if(abs(x) > (delt * 3)) {     // set mouse speed multipler depending on incline
                        x_movement = x_movement * 5;
                    }
                    if(abs(x) > (delt * 4)) {
                        x_movement = x_movement * 5;
                    }
                    if(abs(y) > (delt * 3)) {
                        y_movement = y_movement * 5;
                    }
                    if(abs(y) > (delt * 4)) {
                        y_movement = y_movement * 5;
                    }

                    if((x > delt) && ((y < delt) && (y > -delt))) {    // tilted left
                        ESP_LOGI(TAG, "LEFT");
                        y_movement = 0;
                        esp_hidd_send_mouse_value(hid_conn_id, buttons_state, -x_movement, y_movement);
                    } else if ((x > delt) && (y < -delt)) {
                        ESP_LOGI(TAG, "DOWN LEFT");
                        esp_hidd_send_mouse_value(hid_conn_id, buttons_state, -x_movement, y_movement);
                    } else if ((x > delt) && (y > delt)) {
                        ESP_LOGI(TAG, "UP LEFT");
                        esp_hidd_send_mouse_value(hid_conn_id, buttons_state, -x_movement, -y_movement);
                    } else if ((x < -delt) && ((y < delt) && (y > -delt))) {    // tilted right
                        ESP_LOGI(TAG, "RIGHT");
                        y_movement = 0;
                        esp_hidd_send_mouse_value(hid_conn_id, buttons_state, x_movement, y_movement);
                    } else if ((x < -delt) && (y < -delt)) {
                        ESP_LOGI(TAG, "DOWN RIGHT");
                        esp_hidd_send_mouse_value(hid_conn_id, buttons_state, x_movement, y_movement);
                    } else if ((x < -delt) && (y > delt)) {
                        ESP_LOGI(TAG, "UP RIGHT");
                        esp_hidd_send_mouse_value(hid_conn_id, buttons_state, x_movement, -y_movement);
                    } else if ((y < -delt) && ((x < delt) && (x > -delt))) {     // tilted down
                        ESP_LOGI(TAG, "DOWN");
                        x_movement = 0;
                        esp_hidd_send_mouse_value(hid_conn_id, buttons_state, x_movement, y_movement);
                    } else if ((y > delt) && ((x < delt) && (x > -delt))) {     // tilted up
                        ESP_LOGI(TAG, "UP");
                        x_movement = 0;
                        esp_hidd_send_mouse_value(hid_conn_id, buttons_state, x_movement, -y_movement);
                    } else {
                        ESP_LOGI(TAG, "FLAT");
                        buttons_state = 0x01;
                        esp_hidd_send_mouse_value(hid_conn_id, buttons_state, 0, 0);
                        vTaskDelay(5 / portTICK_PERIOD_MS); // Wait for 1 second if not connected
                        buttons_state = 0x00;
                        esp_hidd_send_mouse_value(hid_conn_id, buttons_state, 0, 0);
                    }

                } else {printf("error reading sensor data\n");}
                vTaskDelay(pdMS_TO_TICKS(100));
            }
        } else {
            vTaskDelay(1000 / portTICK_PERIOD_MS); // Wait for 1 second if not connected
        }
    }
}


void app_main(void)
{
    i2c_master_init();
    
    write_register(0x1F, 0x0F);
    write_register(0x20, 0x66);

    xTaskCreate(imu_task, "imu_task", 2048, NULL, 5, NULL);

    esp_err_t ret;

    // Initialize NVS.
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );

    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret) {
        ESP_LOGE(HID_DEMO_TAG, "%s initialize controller failed", __func__);
        return;
    }

    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret) {
        ESP_LOGE(HID_DEMO_TAG, "%s enable controller failed", __func__);
        return;
    }

    esp_bluedroid_config_t bluedroid_cfg = BT_BLUEDROID_INIT_CONFIG_DEFAULT();
    ret = esp_bluedroid_init_with_cfg(&bluedroid_cfg);
    if (ret) {
        ESP_LOGE(HID_DEMO_TAG, "%s init bluedroid failed", __func__);
        return;
    }

    ret = esp_bluedroid_enable();
    if (ret) {
        ESP_LOGE(HID_DEMO_TAG, "%s init bluedroid failed", __func__);
        return;
    }

    if((ret = esp_hidd_profile_init()) != ESP_OK) {
        ESP_LOGE(HID_DEMO_TAG, "%s init bluedroid failed", __func__);
    }

    ///register the callback function to the gap module
    esp_ble_gap_register_callback(gap_event_handler);
    esp_hidd_register_callbacks(hidd_event_callback);

    /* set the security iocap & auth_req & key size & init key response key parameters to the stack*/
    esp_ble_auth_req_t auth_req = ESP_LE_AUTH_BOND;     //bonding with peer device after authentication
    esp_ble_io_cap_t iocap = ESP_IO_CAP_NONE;           //set the IO capability to No output No input
    uint8_t key_size = 16;      //the key size should be 7~16 bytes
    uint8_t init_key = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;
    uint8_t rsp_key = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;
    esp_ble_gap_set_security_param(ESP_BLE_SM_AUTHEN_REQ_MODE, &auth_req, sizeof(uint8_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_IOCAP_MODE, &iocap, sizeof(uint8_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_MAX_KEY_SIZE, &key_size, sizeof(uint8_t));
    /* If your BLE device act as a Slave, the init_key means you hope which types of key of the master should distribute to you,
    and the response key means which key you can distribute to the Master;
    If your BLE device act as a master, the response key means you hope which types of key of the slave should distribute to you,
    and the init key means which key you can distribute to the slave. */
    esp_ble_gap_set_security_param(ESP_BLE_SM_SET_INIT_KEY, &init_key, sizeof(uint8_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_SET_RSP_KEY, &rsp_key, sizeof(uint8_t));

    xTaskCreate(&hid_demo_task, "hid_task", 2048, NULL, 5, NULL);
}
