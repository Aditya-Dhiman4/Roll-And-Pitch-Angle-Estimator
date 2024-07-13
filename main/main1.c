// #include <stdio.h>
// #include <math.h>
// #include "freertos/FreeRTOS.h"
// #include "freertos/task.h"
// #include "esp_system.h"

// #include "esp_log.h"
// #include "nvs_flash.h"
// #include "driver/i2c_master.h"

// // MPU6050 communication addresses
// #define MPU6050_ADDR 0x68
// #define SDA_GPIO 21
// #define SCL_GPIO 22

// // Accelerometer register addresses
// #define ACCEL_XOUT_H 0x3B
// #define ACCEL_XOUT_L 0x3C
// #define ACCEL_YOUT_H 0x3D
// #define ACCEL_YOUT_L 0x3E
// #define ACCEL_ZOUT_H 0x3F
// #define ACCEL_ZOUT_L 0x40

// // Gyroscope register addresses
// #define GYRO_XOUT_H 0x43
// #define GYRO_XOUT_L 0x44
// #define GYRO_YOUT_H 0x45
// #define GYRO_YOUT_L 0x46
// #define GYRO_ZOUT_H 0x47
// #define GYRO_ZOUT_L 0x48

// #define CALIBRATION_SAMPLES 1000

// #define dt 0.02f  // 20ms in seconds
// #define COMPLEMENTARY_FILTER_ALPHA 0.98f

// const float accel_scale = (2.0 / 32768) * 9.81;
// const float gyro_scale = (250.0 / 32768) * (M_PI / 180.0);
// const float lpf_alpha = 0.05;

// float low_pass_filter(float old_val, float new_val, float alpha) {
//     return alpha * new_val + (1 - alpha) * old_val;
// }

// void calibrate(i2c_master_dev_handle_t device_handle, float * offsets) {
//     float accel_x_sum = 0, accel_y_sum = 0, accel_z_sum = 0;
//     float gyro_x_sum = 0, gyro_y_sum = 0, gyro_z_sum = 0;

//     for (int i = 0; i < CALIBRATION_SAMPLES; i++) {
//         uint8_t accel_data[6], gyro_data[6];
//         uint8_t accel_reg_addr = ACCEL_XOUT_H;
//         uint8_t gyro_reg_addr = GYRO_XOUT_H;

//         ESP_ERROR_CHECK(i2c_master_transmit_receive(device_handle, &accel_reg_addr, 1, accel_data, sizeof(accel_data), -1));
//         ESP_ERROR_CHECK(i2c_master_transmit_receive(device_handle, &gyro_reg_addr, 1, gyro_data, sizeof(gyro_data), -1));

//         int16_t accel_x = ((accel_data[0] << 8) | accel_data[1]);
//         int16_t accel_y = (accel_data[2] << 8) | accel_data[3];
//         int16_t accel_z = (accel_data[4] << 8) | accel_data[5];

//         int16_t gyro_x = (gyro_data[0] << 8) | gyro_data[1];
//         int16_t gyro_y = (gyro_data[2] << 8) | gyro_data[3];
//         int16_t gyro_z = (gyro_data[4] << 8) | gyro_data[5];

//         accel_x_sum += accel_x * accel_scale;
//         accel_y_sum += accel_y * accel_scale;
//         accel_z_sum += (accel_z * accel_scale) - 9.81;

//         gyro_x_sum += gyro_x * gyro_scale;
//         gyro_y_sum += gyro_y * gyro_scale;
//         gyro_z_sum += gyro_z * gyro_scale;

//         // ESP_LOGI("Calibration", "ACCEL - X: %.2f m\s^2, Y: %.2f m\s^2, Z: %.2f m\s^2 --- GYRO - X: %.2f°, Y: %.2f°, Z: %.2f°", accel_x * accel_scale, accel_y * accel_scale, accel_z * accel_scale, gyro_x * gyro_scale, gyro_y * gyro_scale, gyro_z * gyro_scale);

//         vTaskDelay(1 / portTICK_PERIOD_MS);
//     }

//     offsets[0] = accel_x_sum / CALIBRATION_SAMPLES;
//     offsets[1] = accel_y_sum / CALIBRATION_SAMPLES;
//     offsets[2] = accel_z_sum / CALIBRATION_SAMPLES; // Assuming Z-axis is vertical

//     offsets[3] = gyro_x_sum / CALIBRATION_SAMPLES;
//     offsets[4] = gyro_y_sum / CALIBRATION_SAMPLES;
//     offsets[5] = gyro_z_sum / CALIBRATION_SAMPLES;

//     ESP_LOGI("Calibration", "Accel offsets: X=%.4f, Y=%.4f, Z=%.4f", offsets[0], offsets[1], offsets[2]);
//     ESP_LOGI("Calibration", "Gyro offsets: X=%.4f, Y=%.4f, Z=%.4f", offsets[3], offsets[4], offsets[5]);
// }

// void app_main(void)
// {
//     char *task_name = pcTaskGetName(NULL);
//     ESP_LOGI(task_name, "Starting up task...\n");

//     i2c_master_bus_config_t bus_config = {
//         .i2c_port = I2C_NUM_0,
//         .sda_io_num = SDA_GPIO,
//         .scl_io_num = SCL_GPIO,
//         .clk_source = I2C_CLK_SRC_DEFAULT,
//         .glitch_ignore_cnt = 7,
//         .flags.enable_internal_pullup = true
//     };

//     i2c_device_config_t device_config = {
//         .dev_addr_length = I2C_ADDR_BIT_LEN_7,
//         .device_address = MPU6050_ADDR,
//         .scl_speed_hz = 100000
//     };

//     i2c_master_bus_handle_t bus_handle;
//     ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, &bus_handle));
//     ESP_LOGI(task_name, "Installed drivers\n");

//     i2c_master_dev_handle_t device_handle;
//     ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &device_config, &device_handle));
//     ESP_LOGI(task_name, "Added device\n");

//     uint8_t wr_buf[] = {0x6B, 0x00};
//     ESP_ERROR_CHECK(i2c_master_transmit(device_handle, wr_buf, sizeof(wr_buf), -1));
//     ESP_LOGI(task_name, "Requested to wake sensor\n");

//     uint8_t reg_addr = 0x6B;
//     uint8_t data;
//     ESP_ERROR_CHECK(i2c_master_transmit_receive(device_handle, &reg_addr, 1, &data, 1, -1));

//     if (data == 0x00) {
//             ESP_LOGI(task_name, "Successfully woke sensor: 0x%02X\n", data);
//         } else {
//             ESP_LOGI(task_name, "Failed to wake up sensor: 0x%02X\n", data);
//         }
    
//     vTaskDelay(3000 / portTICK_PERIOD_MS);

//     uint8_t accel_conf_buf[] = {0x1C, 0x00};
//     ESP_ERROR_CHECK(i2c_master_transmit(device_handle, accel_conf_buf, sizeof(accel_conf_buf), -1));
//     ESP_LOGI(task_name, "Setting accelerometer to 2g\n");

//     uint8_t gryo_conf_buf[] = {0x1B, 0x00};
//     ESP_ERROR_CHECK(i2c_master_transmit(device_handle, gryo_conf_buf, sizeof(gryo_conf_buf), -1));
//     ESP_LOGI(task_name, "Setting gyroscope to 250*/s");

//     uint8_t accel_reg_addr = ACCEL_XOUT_H;
//     uint8_t gyro_reg_addr = GYRO_XOUT_H;

//     uint8_t accel_data[6];
//     uint8_t gyro_data[6];

//     float accel_xf = 0, accel_yf = 0, accel_zf = 0, accel_xf_prev = 0, accel_yf_prev = 0, accel_zf_prev = 0;
//     float gyro_xf = 0, gyro_yf = 0, gyro_zf = 0, gyro_xf_prev = 0, gyro_yf_prev = 0, gyro_zf_prev = 0;
//     float angle_x = 0, angle_y = 0, angle_z = 0, angle_x_prev = 0, angle_y_prev = 0, angle_z_prev = 0;
//     float accel_total = 0, gyro_total = 0;

//     float offsets[6];

//     ESP_LOGI("Calibration", "Starting Calibration...");
//     calibrate(device_handle, &offsets);
//     ESP_LOGI("Calibration", "Finished Calibration!");

//     float forearm_length = 0, prev_forearm_length = 0;

//     vTaskDelay(3000 / portTICK_PERIOD_MS);

//     while (1) {
//         ESP_ERROR_CHECK(i2c_master_transmit_receive(device_handle, &accel_reg_addr, 1, accel_data, sizeof(accel_data), -1));

//         int16_t accel_x = ((accel_data[0] << 8) | accel_data[1]);
//         int16_t accel_y = (accel_data[2] << 8) | accel_data[3];
//         int16_t accel_z = (accel_data[4] << 8) | accel_data[5];

//         ESP_ERROR_CHECK(i2c_master_transmit_receive(device_handle, &gyro_reg_addr, 1, gyro_data, sizeof(gyro_data), -1));

//         int16_t gyro_x = (gyro_data[0] << 8) | gyro_data[1];
//         int16_t gyro_y = (gyro_data[2] << 8) | gyro_data[3];
//         int16_t gyro_z = (gyro_data[4] << 8) | gyro_data[5];

//         accel_xf = low_pass_filter(accel_xf_prev, (accel_x * accel_scale) - offsets[0], lpf_alpha);
//         accel_yf = low_pass_filter(accel_yf_prev, (accel_y * accel_scale) - offsets[1], lpf_alpha);
//         accel_zf = low_pass_filter(accel_zf_prev, (accel_z * accel_scale) - offsets[2], lpf_alpha);


//         gyro_xf = low_pass_filter(gyro_xf_prev, (gyro_x * gyro_scale) - offsets[3], lpf_alpha);
//         gyro_yf = low_pass_filter(gyro_yf_prev, (gyro_y * gyro_scale) - offsets[4], lpf_alpha);
//         gyro_zf = low_pass_filter(gyro_zf_prev, (gyro_z * gyro_scale) - offsets[5], lpf_alpha);

//         float accel_angle_x = atan2f(accel_yf, accel_zf);
//         float accel_angle_y = atan2f(-accel_xf, sqrtf(accel_yf*accel_yf + accel_zf*accel_zf));

//         angle_x = COMPLEMENTARY_FILTER_ALPHA * (angle_x + gyro_xf * dt) + (1 - COMPLEMENTARY_FILTER_ALPHA) * accel_angle_x;
//         angle_y = COMPLEMENTARY_FILTER_ALPHA * (angle_y + gyro_yf * dt) + (1 - COMPLEMENTARY_FILTER_ALPHA) * accel_angle_y;
//         angle_z += gyro_zf * dt;  // We can't correct Z rotation with accelerometer

//         angle_x = atan2f(sinf(angle_x), cosf(angle_x));
//         angle_y = atan2f(sinf(angle_y), cosf(angle_y));
//         angle_z = atan2f(sinf(angle_z), cosf(angle_z));

//         // accel_total = sqrt((accel_xf * accel_xf) + (accel_yf * accel_yf) + ((accel_zf - 9.81f) * (accel_zf - 9.81f)));
//         // gyro_total = sqrt((gyro_xf * gyro_xf) + (gyro_yf * gyro_yf) + (gyro_zf * gyro_zf));

//         // forearm_length = (gyro_total > 0.02f) ? accel_total / (gyro_total*gyro_total) : 0;

//         // if (forearm_length > 0.6) {
//         //     forearm_length = 0.6;
//         // } else if (forearm_length < 0.1) {
//         //     forearm_length = 0.1;
//         // }

//         // forearm_length = low_pass_filter(prev_forearm_length, forearm_length, lpf_alpha);

//         // ESP_LOGI(task_name, "ACCEL - X: %.2f g, Y: %.2f g, Z: %.2f g --- GYRO - X: %.2f rad/s, Y: %.2f rad/s, Z: %.2f rad/s", accel_xf, accel_yf, accel_zf, gyro_xf, gyro_yf, gyro_zf);
//         ESP_LOGI(task_name, "ACCEL - X: %.2f m/s^2, Y: %.2f m/s^2, Z: %.2f m/s^2 --- ANGLE - X: %.2f°, Y: %.2f°, Z: %.2f°", accel_xf, accel_yf, accel_zf - 9.81, angle_x * 180.0f / M_PI, angle_y * 180.0f / M_PI, angle_z * 180.0f / M_PI);
//         // ESP_LOGI(task_name, "Radial ACCEL: %.2f g --- Radial GYRO: %.2f rad/s, forearm length: %.2f m", accel_total, gyro_total, forearm_length);

//         accel_xf_prev = accel_xf;
//         accel_yf_prev = accel_yf;
//         accel_zf_prev = accel_zf;

//         gyro_xf_prev = gyro_xf;
//         gyro_yf_prev = gyro_yf;
//         gyro_zf_prev = gyro_zf;

//         // angle_x_prev = angle_x;
//         // angle_y_prev = angle_y;
//         // angle_z_prev = angle_z;

//         // prev_forearm_length = forearm_length;

//         vTaskDelay(20 / portTICK_PERIOD_MS);
//     }
// }