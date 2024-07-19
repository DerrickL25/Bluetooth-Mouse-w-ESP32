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

void app_main() {
    i2c_master_init();
    
    write_register(0x1F, 0x0F);
    write_register(0x20, 0x66);

    xTaskCreate(imu_task, "imu_task", 2048, NULL, 5, NULL);
}
