#include "mpu6050.h"

static const char *TAG = "MPU6050";

static const float accel_scale = (2.0 / 32768) * 9.81;
static const float gyro_scale = (250.0 / 32768) * (M_PI / 180.0);

esp_err_t mpu6050_init(mpu6050_t *mpu6050) {
    i2c_master_bus_config_t bus_config = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = SDA_GPIO,
        .scl_io_num = SCL_GPIO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true
    };

    i2c_device_config_t device_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = MPU6050_ADDR,
        .scl_speed_hz = 100000
    };

    i2c_master_bus_handle_t bus_handle;
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, &bus_handle));
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &device_config, &mpu6050->device_handle));

    uint8_t reg_addr = 0x6B;
    uint8_t wr_buf[] = {reg_addr, 0x00};
    ESP_ERROR_CHECK(i2c_master_transmit(mpu6050->device_handle, wr_buf, sizeof(wr_buf), -1));

    uint8_t data;
    ESP_ERROR_CHECK(i2c_master_transmit_receive(mpu6050->device_handle, &reg_addr, 1, &data, 1, -1));

    if (data == 0x00) {
        ESP_LOGI(TAG, "Successfully woke sensor: 0x%02X", data);
    } else {
        ESP_LOGI(TAG, "Failed to wake up sensor: 0x%02X", data);
        return ESP_FAIL;
    }

    uint8_t accel_conf_buf[] = {0x1C, 0x00};
    ESP_ERROR_CHECK(i2c_master_transmit(mpu6050->device_handle, accel_conf_buf, sizeof(accel_conf_buf), -1));

    uint8_t gyro_conf_buf[] = {0x1B, 0x00};
    ESP_ERROR_CHECK(i2c_master_transmit(mpu6050->device_handle, gyro_conf_buf, sizeof(gyro_conf_buf), -1));

    return ESP_OK;
}

void mpu6050_calibrate(mpu6050_t *mpu6050) {
    float accel_sum[3] = {0}, gyro_sum[3] = {0};

    for (int i = 0; i < CALIBRATION_SAMPLES; i++) {
        uint8_t accel_data[6], gyro_data[6];
        uint8_t accel_reg_addr = ACCEL_XOUT_H;
        uint8_t gyro_reg_addr = GYRO_XOUT_H;

        ESP_ERROR_CHECK(i2c_master_transmit_receive(mpu6050->device_handle, &accel_reg_addr, 1, accel_data, sizeof(accel_data), -1));
        ESP_ERROR_CHECK(i2c_master_transmit_receive(mpu6050->device_handle, &gyro_reg_addr, 1, gyro_data, sizeof(gyro_data), -1));

        for (int j = 0; j < 3; j++) {
            int16_t accel = (accel_data[j*2] << 8) | accel_data[j*2 + 1];
            int16_t gyro = (gyro_data[j*2] << 8) | gyro_data[j*2 + 1];

            accel_sum[j] += accel * accel_scale;
            gyro_sum[j] += gyro * gyro_scale;
        }

        accel_sum[2] -= 9.81;

        vTaskDelay(1 / portTICK_PERIOD_MS);
    }

    for (int i = 0; i < 3; i++) {
        mpu6050->accel_offsets[i] = accel_sum[i] / CALIBRATION_SAMPLES;
        mpu6050->gyro_offsets[i] = gyro_sum[i] / CALIBRATION_SAMPLES;
    }

    ESP_LOGI(TAG, "Accel offsets: X=%.4f, Y=%.4f, Z=%.4f", mpu6050->accel_offsets[0], mpu6050->accel_offsets[1], mpu6050->accel_offsets[2]);
    ESP_LOGI(TAG, "Gyro offsets: X=%.4f, Y=%.4f, Z=%.4f", mpu6050->gyro_offsets[0], mpu6050->gyro_offsets[1], mpu6050->gyro_offsets[2]);
}

void mpu6050_read_data(mpu6050_t *mpu6050, float *accel, float *gyro) {
    uint8_t accel_data[6], gyro_data[6];
    uint8_t accel_reg_addr = ACCEL_XOUT_H;
    uint8_t gyro_reg_addr = GYRO_XOUT_H;

    ESP_ERROR_CHECK(i2c_master_transmit_receive(mpu6050->device_handle, &accel_reg_addr, 1, accel_data, sizeof(accel_data), -1));
    ESP_ERROR_CHECK(i2c_master_transmit_receive(mpu6050->device_handle, &gyro_reg_addr, 1, gyro_data, sizeof(gyro_data), -1));

    for (int i = 0; i < 3; i++) {
        int16_t accel_raw = (accel_data[i*2] << 8) | accel_data[i*2 + 1];
        int16_t gyro_raw = (gyro_data[i*2] << 8) | gyro_data[i*2 + 1];

        accel[i] = (accel_raw * accel_scale) - mpu6050->accel_offsets[i];
        gyro[i] = (gyro_raw * gyro_scale) - mpu6050->gyro_offsets[i];
    }
}