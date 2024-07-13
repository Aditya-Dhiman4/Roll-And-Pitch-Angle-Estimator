| Supported Targets | ESP32 |
| ----------------- | ----- | -------- | -------- | -------- | -------- | -------- | -------- | -------- | -------- |

# MPU6050 HAL

Note: This has not been tested on the following boards and may not work on these boards:

ESP32-C2 | ESP32-C3 | ESP32-C5 | ESP32-C6 | ESP32-H2 | ESP32-P4 | ESP32-S2 | ESP32-S3 |

This example is a Hardware Abstraction Layer for the MPU6050 IMU Sensor. The MPU6050 IMU contains an accelerometer and gyroscope, allowing for 6 degrees of freedom. It uses a calibration sequence and a low pass filter to allow for smoother more accurate data. For more information follow the [docs page](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/peripherals/i2c.html)

## How to use example

Download or clone into any folder and run `idf.py build`, `idf.py flash`, `idf.py monitor`

Select the instructions depending on Espressif chip installed on your development board:

- [ESP32 Getting Started Guide](https://docs.espressif.com/projects/esp-idf/en/stable/get-started/index.html)
- [ESP32-S2 Getting Started Guide](https://docs.espressif.com/projects/esp-idf/en/latest/esp32s2/get-started/index.html)

## Example folder contents

The project **MPU6050_HAL** contains three source files in C language: [main.c](main/main.c), [mpu6050.c](main/mpu6050.c), and [mpu6050.h](main/mpu6050.h). The files are located in folder [main](main).

ESP-IDF projects are built using CMake. The project build configuration is contained in `CMakeLists.txt`
files that provide set of directives and instructions describing the project's source files and targets
(executable, library, or both). 

Below is short explanation of remaining files in the project folder.

```
├── CMakeLists.txt
├── main
│   ├── CMakeLists.txt
    ├── main.c
    ├── mpu6050.c
│   └── mpu6050.h
└── README.md                  This is the file you are currently reading
```



