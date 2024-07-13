#include "mpu6050.h"

float low_pass_filter(float old_val, float new_val, float alpha) {
    return alpha * new_val + (1 - alpha) * old_val;
}

void app_main(void)
{
    mpu6050_t mpu6050;
    
    ESP_ERROR_CHECK(mpu6050_init(&mpu6050));
    
    ESP_LOGI("Calibration", "Starting Calibration...");
    mpu6050_calibrate(&mpu6050);
    ESP_LOGI("Calibration", "Finished Calibration!");

    vTaskDelay(3000 / portTICK_PERIOD_MS);

    float accel[3] = {0}, gyro[3] = {0};
    float accel_prev[3] = {0}, gyro_prev[3] = {0};

    while (1) {
        mpu6050_read_data(&mpu6050, accel, gyro);

        for (int i = 0; i < 3; i++) {
            accel[i] = low_pass_filter(accel_prev[i], accel[i], 0.05);
            gyro[i] = low_pass_filter(gyro_prev[i], gyro[i], 0.05);

            accel_prev[i] = accel[i];
            gyro_prev[i] = gyro[i];
        }

            ESP_LOGI("MPU6050", "ACCEL - X: %.2f m/s^2, Y: %.2f m/s^2, Z: %.2f m/s^2 --- GYRO - X: %.2f rad/s, Y: %.2f rad/s, Z: %.2f rad/s",
                     accel[0], accel[1], accel[2], gyro[0], gyro[1], gyro[2]);

        vTaskDelay(20 / portTICK_PERIOD_MS);
    }
}