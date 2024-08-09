#include "mpu6050.h"

#define SAMPLE_TIME_MS 20

// Low Pass Filter
// Returns Filtered Value
// Takes in old value and new value and returns filtered value
float low_pass_filter(float old_val, float new_val, float alpha) {
    return alpha * new_val + (1 - alpha) * old_val;
}

// 1D Kalman Filter - https://en.wikipedia.org/wiki/
// Returns filtered value from input and measurement
// 1. Calculate state by integration of input from sensor
// 2. Calculate uncertaincy of input from sensor
// 3. Calculate kalman gain
// 4. Calcualte new state and uncerstancy using gain, previous state, and measurement
//
//
// Needs to be tuned to be more responsive
void kalman_filter(float state, float uncertaincy, float input, float measurement, float* out) {
    state += (SAMPLE_TIME_MS / 1000) * input;
    uncertaincy += (SAMPLE_TIME_MS / 1000)*(SAMPLE_TIME_MS / 1000)*4*4;
    float gain = uncertaincy / (uncertaincy + 3*3);
    state += gain * (measurement - state);
    uncertaincy = (1 - gain) * uncertaincy;
    out[0] = state;
    out[1] = uncertaincy;
    ESP_LOGI("Kalman", "State: %.4f, Uncertaincy: %.4f, Gain: %.4f", state, uncertaincy, gain);
}

void app_main(void)
{
    mpu6050_t mpu6050;
    
    ESP_ERROR_CHECK(mpu6050_init(&mpu6050));
    
    ESP_LOGI("Calibration", "Starting Calibration...");
    mpu6050_calibrate(&mpu6050);
    ESP_LOGI("Calibration", "Finished Calibration!");

    vTaskDelay(3000 / portTICK_PERIOD_MS);

    float accel[3] = {0}, gyro[3] = {0}, angle_ac[2] = {0};

    // float kalman_angle_x = 0, kalman_angle_y = 0;
    // float kalman_uncertaincy_x = 4, kalman_uncertaincy_y = 4;
    // float kalman_out[] = {0, 0};

    float gyro_dot[2] = {0}, gyro_final[2] = {0};
    float accel_prev[3] = {0}, gyro_prev[3] = {0}, gyro_c_prev[2] = {0};

    while (1) {
        mpu6050_read_data(&mpu6050, accel, gyro);

        // Calculating angle from accelerometer
        angle_ac[0] = atan2f(accel[1], sqrtf(accel[0]*accel[0] + accel[2]*accel[2])) * (180.0f / M_PI);
        angle_ac[1] = atan2f(-accel[0], sqrtf(accel[1]*accel[1] + accel[2]*accel[2])) * (180.0f / M_PI);

        // Kalman Filter Code

        // kalman_filter(kalman_angle_x, kalman_uncertaincy_x, gyro[0] * (180.0f / M_PI), angle_ac[0], kalman_out);

        // kalman_angle_x = kalman_out[0];
        // kalman_uncertaincy_x = kalman_out[1];

        // kalman_filter(kalman_angle_y, kalman_uncertaincy_y, gyro[1] * (180.0f / M_PI), angle_ac[1], kalman_out);

        // kalman_angle_y = kalman_out[0];
        // kalman_uncertaincy_y = kalman_out[1];

        // ESP_LOGI("MPU6050", "ACCEL - X: %.2f m/s^2, Y: %.2f m/s^2, Z: %.2f m/s^2 --- GYRO - X: %.2f rad/s, Y: %.2f rad/s, Z: %.2f rad/s", accel[0], accel[1], accel[2], gyro[0], gyro[1], gyro[2]);

        // Complementary Filter Code

        gyro_dot[0] = gyro[0] + tanf(gyro_final[1]) * (gyro[1] * sinf(gyro_final[0]) + gyro[2] * cosf(gyro_final[0]));
        gyro_dot[1] = gyro[1] * cosf(gyro_final[0]) - gyro[2] * sinf(gyro_final[1]);

        gyro_final[0] = 0.05f * angle_ac[0] + (1.0f - 0.05f) * (gyro_final[0] + (SAMPLE_TIME_MS / 1000.0f) * gyro_dot[0]);
        gyro_final[1] = 0.05f * angle_ac[1] + (1.0f - 0.05f) * (gyro_final[1] + (SAMPLE_TIME_MS / 1000.0f) * gyro_dot[1]);

        ESP_LOGI("MPU6050", "ANGLE - X: %.2f, Y: %.2f", gyro_final[0], gyro_final[1]);

        vTaskDelay(SAMPLE_TIME_MS / portTICK_PERIOD_MS);
    }
}