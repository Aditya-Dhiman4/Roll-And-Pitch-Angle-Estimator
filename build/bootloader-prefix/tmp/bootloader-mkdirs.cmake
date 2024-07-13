# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "C:/esp/esp-idf/components/bootloader/subproject"
  "C:/programs/cpp/esp32/imu-test-1/build/bootloader"
  "C:/programs/cpp/esp32/imu-test-1/build/bootloader-prefix"
  "C:/programs/cpp/esp32/imu-test-1/build/bootloader-prefix/tmp"
  "C:/programs/cpp/esp32/imu-test-1/build/bootloader-prefix/src/bootloader-stamp"
  "C:/programs/cpp/esp32/imu-test-1/build/bootloader-prefix/src"
  "C:/programs/cpp/esp32/imu-test-1/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "C:/programs/cpp/esp32/imu-test-1/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "C:/programs/cpp/esp32/imu-test-1/build/bootloader-prefix/src/bootloader-stamp${cfgdir}") # cfgdir has leading slash
endif()
