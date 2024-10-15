#include <stdio.h>
#include "driver/i2c.h"
#include "esp_log.h"
#include "esp_system.h"

// I2C configuration
#define I2C_MASTER_SCL_IO           22      // GPIO for SCL
#define I2C_MASTER_SDA_IO           21      // GPIO for SDA
#define I2C_MASTER_NUM              I2C_NUM_0
#define I2C_MASTER_FREQ_HZ          100000  // I2C frequency
#define I2C_MASTER_TX_BUF_DISABLE   0
#define I2C_MASTER_RX_BUF_DISABLE   0
#define MPU6050_SENSOR_ADDR         0x68    // MPU6050 I2C address
#define MPU6050_WHO_AM_I_REG        0x75    // WHO_AM_I register address
#define MPU6050_PWR_MGMT_1_REG      0x6B    // Power management register
#define MPU6050_ACCEL_XOUT_H        0x3B    // Register to start reading accelerometer data

// Function to initialize the I2C
static esp_err_t i2c_master_init(void) {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    i2c_param_config(I2C_MASTER_NUM, &conf);
    return i2c_driver_install(I2C_MASTER_NUM, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}

// Function to write to the MPU-6050 register
static esp_err_t mpu6050_write_byte(uint8_t reg_addr, uint8_t data) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU6050_SENSOR_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);
    i2c_master_write_byte(cmd, data, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

// Function to read from the MPU-6050 register
static esp_err_t mpu6050_read(uint8_t reg_addr, uint8_t *data, size_t len) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU6050_SENSOR_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU6050_SENSOR_ADDR << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd, data, len, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

// Function to initialize the MPU-6050
static void mpu6050_init() {
    // Wake up the MPU-6050 as it starts in sleep mode
    mpu6050_write_byte(MPU6050_PWR_MGMT_1_REG, 0x00);
}

// Function to read accelerometer data
static void read_accel_data() {
    uint8_t data[6];
    mpu6050_read(MPU6050_ACCEL_XOUT_H, data, 6);

    int16_t accel_x = (data[0] << 8) | data[1];
    int16_t accel_y = (data[2] << 8) | data[3];
    int16_t accel_z = (data[4] << 8) | data[5];

    printf("Accel X: %d, Accel Y: %d, Accel Z: %d\n", accel_x, accel_y, accel_z);
}

void app_main(void) {
    // Initialize I2C
    ESP_ERROR_CHECK(i2c_master_init());

    // Initialize MPU-6050
    mpu6050_init();

    // Read and print data in a loop
    while (1) {
        read_accel_data();
        vTaskDelay(500 / portTICK_PERIOD_MS);
    }
}
