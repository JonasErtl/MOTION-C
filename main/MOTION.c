#include <stdio.h>
#include <math.h>
#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define I2C_MASTER_SCL_IO           22
#define I2C_MASTER_SDA_IO           21
#define I2C_MASTER_NUM              I2C_NUM_0
#define I2C_MASTER_FREQ_HZ          100000
#define MPU6050_SENSOR_ADDR         0x68
#define MPU6050_PWR_MGMT_1_REG      0x6B
#define MPU6050_ACCEL_XOUT_H        0x3B
#define ACCEL_SENSITIVITY 16384.0  // Accelerometer Sensitivity for ±2g 
#define GYRO_SENSITIVITY 131.0 // gyro sensitivity 
#define NUM_SAMPLES 100 // samples for calibration
//inital offsets for calibration 
int16_t accel_x_offset = 0, accel_y_offset = 0, accel_z_offset = 0;
int16_t gyro_x_offset = 0, gyro_y_offset = 0, gyro_z_offset = 0;

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
    return i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
}

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

static void mpu6050_init() {
    mpu6050_write_byte(MPU6050_PWR_MGMT_1_REG, 0x00);  
}

void calibrate_accelerometer() {
    int32_t accel_x_sum = 0, accel_y_sum = 0, accel_z_sum = 0;

    printf("Starting accelerometer calibration...\n");

    for (int i = 0; i < NUM_SAMPLES; i++) {
        uint8_t data[6];
        mpu6050_read(MPU6050_ACCEL_XOUT_H, data, 6);

        int16_t accel_x = (data[0] << 8) | data[1];
        int16_t accel_y = (data[2] << 8) | data[3];
        int16_t accel_z = (data[4] << 8) | data[5];

        accel_x_sum += accel_x;
        accel_y_sum += accel_y;
        accel_z_sum += accel_z;

        vTaskDelay(10 / portTICK_PERIOD_MS);  
    }

    accel_x_offset = accel_x_sum / NUM_SAMPLES;
    accel_y_offset = accel_y_sum / NUM_SAMPLES;
    accel_z_offset = (accel_z_sum / NUM_SAMPLES) - 16384;  

    printf("Calibration complete. Offsets:\n");
    printf("Accel X Offset: %d\n", accel_x_offset);
    printf("Accel Y Offset: %d\n", accel_y_offset);
    printf("Accel Z Offset: %d\n", accel_z_offset);
}

void read_accel_data_in_g() {
    uint8_t data[6];  // Buffer accel data
    mpu6050_read(MPU6050_ACCEL_XOUT_H, data, 6);

    int16_t accel_x_raw = (data[0] << 8) | data[1];
    int16_t accel_y_raw = (data[2] << 8) | data[3];
    int16_t accel_z_raw = (data[4] << 8) | data[5];

    accel_x_raw -= accel_x_offset;
    accel_y_raw -= accel_y_offset;
    accel_z_raw -= accel_z_offset;

    float accel_x_g = accel_x_raw / ACCEL_SENSITIVITY;
    float accel_y_g = accel_y_raw / ACCEL_SENSITIVITY;
    float accel_z_g = accel_z_raw / ACCEL_SENSITIVITY;

    printf("Accel in g: X=%.3f g, Y=%.3f g, Z=%.3f g\n", accel_x_g, accel_y_g, accel_z_g);
}


//funtion currently not in use
void read_calibrated_accel_data() {
    uint8_t data[6];
    mpu6050_read(MPU6050_ACCEL_XOUT_H, data, 6);

    int16_t accel_x = (data[0] << 8) | data[1];
    int16_t accel_y = (data[2] << 8) | data[3];
    int16_t accel_z = (data[4] << 8) | data[5];


    accel_x -= accel_x_offset;
    accel_y -= accel_y_offset;
    accel_z -= accel_z_offset;

    printf("Calibrated Accel Data: X=%d, Y=%d, Z=%d\n", accel_x, accel_y, accel_z);
}

// gyro calibration
void calibrate_gyroscope() {
    int32_t gyro_x_sum = 0, gyro_y_sum = 0, gyro_z_sum = 0;

    printf("Starting gyroscope calibration. Keep the sensor stationary...\n");

    for (int i = 0; i < NUM_SAMPLES; i++) {
        uint8_t data[6];
        mpu6050_read(0x43, data, 6);  // Start reading from GYRO_XOUT_H

        int16_t gyro_x_raw = (data[0] << 8) | data[1];
        int16_t gyro_y_raw = (data[2] << 8) | data[3];
        int16_t gyro_z_raw = (data[4] << 8) | data[5];

        gyro_x_sum += gyro_x_raw;
        gyro_y_sum += gyro_y_raw;
        gyro_z_sum += gyro_z_raw;

        vTaskDelay(10 / portTICK_PERIOD_MS);  // Short delay between readings
    }

    // Calculate offsets as the average of the samples
    gyro_x_offset = gyro_x_sum / NUM_SAMPLES;
    gyro_y_offset = gyro_y_sum / NUM_SAMPLES;
    gyro_z_offset = gyro_z_sum / NUM_SAMPLES;

    printf("Gyro Calibration Complete:\n");
    printf("X Offset: %d, Y Offset: %d, Z Offset: %d\n", gyro_x_offset, gyro_y_offset, gyro_z_offset);
}

void read_calibrated_gyro_data() {
    uint8_t data[6];
    mpu6050_read(0x43, data, 6);

    int16_t gyro_x_raw = (data[0] << 8) | data[1];
    int16_t gyro_y_raw = (data[2] << 8) | data[3];
    int16_t gyro_z_raw = (data[4] << 8) | data[5];

    // Apply calibration offsets
    gyro_x_raw -= gyro_x_offset;
    gyro_y_raw -= gyro_y_offset;
    gyro_z_raw -= gyro_z_offset;

    // Convert to degrees per second
    float gyro_x_dps = gyro_x_raw / GYRO_SENSITIVITY;
    float gyro_y_dps = gyro_y_raw / GYRO_SENSITIVITY;
    float gyro_z_dps = gyro_z_raw / GYRO_SENSITIVITY;

    printf("Gyro in DPS: X=%.3f °/s, Y=%.3f °/s, Z=%.3f °/s\n", gyro_x_dps, gyro_y_dps, gyro_z_dps);
}

//function currently not in use
void read_gyro_data_in_dps() {
    uint8_t data[6];   
    mpu6050_read(0x43, data, 6);  

    // Combining registers to make a 16 Bit int
    int16_t gyro_x_raw = (data[0] << 8) | data[1];
    int16_t gyro_y_raw = (data[2] << 8) | data[3];
    int16_t gyro_z_raw = (data[4] << 8) | data[5];

    float gyro_x_dps = gyro_x_raw / GYRO_SENSITIVITY;
    float gyro_y_dps = gyro_y_raw / GYRO_SENSITIVITY;
    float gyro_z_dps = gyro_z_raw / GYRO_SENSITIVITY;

    printf("Gyro in DPS: X=%.3f °/s, Y=%.3f °/s, Z=%.3f °/s\n", gyro_x_dps, gyro_y_dps, gyro_z_dps);
}

void app_main(void) {
    ESP_ERROR_CHECK(i2c_master_init());
    mpu6050_init();

    calibrate_accelerometer();

 calibrate_gyroscope();
    while (1) {
        read_accel_data_in_g();
        read_calibrated_gyro_data();
        vTaskDelay(500 / portTICK_PERIOD_MS);
    }
}

