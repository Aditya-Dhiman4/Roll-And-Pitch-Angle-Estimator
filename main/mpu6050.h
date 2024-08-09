#ifndef MPU6050_H
#define MPU6050_H

#include <stdio.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/i2c_master.h"

// MPU6050 communication addresses
#define MPU6050_ADDR 0x68
#define SDA_GPIO 21
#define SCL_GPIO 22

// Accelerometer register addresses
#define ACCEL_XOUT_H 0x3B
#define ACCEL_XOUT_L 0x3C
#define ACCEL_YOUT_H 0x3D
#define ACCEL_YOUT_L 0x3E
#define ACCEL_ZOUT_H 0x3F
#define ACCEL_ZOUT_L 0x40

// Gyroscope register addresses
#define GYRO_XOUT_H 0x43
#define GYRO_XOUT_L 0x44
#define GYRO_YOUT_H 0x45
#define GYRO_YOUT_L 0x46
#define GYRO_ZOUT_H 0x47
#define GYRO_ZOUT_L 0x48

#define CALIBRATION_SAMPLES 3000

typedef struct {
    i2c_master_dev_handle_t device_handle;
    float accel_offsets[3];
    float gyro_offsets[3];
} mpu6050_t;

// Initializes the MPU6050 IMU
// Returns ESP_OK if successful
// Initializes i2c bus and device and checks if sensor is communicable
esp_err_t mpu6050_init(mpu6050_t *mpu6050);

// Calibrates the MPU6050 IMU
// Communicates through the i2c bus to get accelerometer and gyroscope readings
// Sums accelerometer and gyroscope readings over CALIBRATION_SAMPLES per millisecond
// and stores the result in the mpu6050_t struct
void mpu6050_calibrate(mpu6050_t *mpu6050);

// Obtains Calibrated Data from Accelerometer and Gyroscope of the MPU6050 IMU
// Communicates through the i2c bus to get accelerometer and gyroscope readings
// subtracts calibration offsets from raw data
// Returns the calibrated data in the float array
void mpu6050_read_data(mpu6050_t *mpu6050, float *accel, float *gyro);

#endif // MPU6050_H