# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "D:/SoftWare/Espressif/frameworks/esp-idf-v5.1.2/components/bootloader/subproject"
  "D:/esp-code/espidf-st7796-spi/spi-lcd-touch-ok/build/bootloader"
  "D:/esp-code/espidf-st7796-spi/spi-lcd-touch-ok/build/bootloader-prefix"
  "D:/esp-code/espidf-st7796-spi/spi-lcd-touch-ok/build/bootloader-prefix/tmp"
  "D:/esp-code/espidf-st7796-spi/spi-lcd-touch-ok/build/bootloader-prefix/src/bootloader-stamp"
  "D:/esp-code/espidf-st7796-spi/spi-lcd-touch-ok/build/bootloader-prefix/src"
  "D:/esp-code/espidf-st7796-spi/spi-lcd-touch-ok/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "D:/esp-code/espidf-st7796-spi/spi-lcd-touch-ok/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "D:/esp-code/espidf-st7796-spi/spi-lcd-touch-ok/build/bootloader-prefix/src/bootloader-stamp${cfgdir}") # cfgdir has leading slash
endif()
