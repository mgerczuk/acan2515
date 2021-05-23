idf_component_register(SRCS "src/ACAN2515.cpp" "src/ACAN2515Settings.cpp" "src/esp32-hal-cpu.c" "src/esp32-hal-gpio.c"
                            "src/esp32-hal.c" "src/esp32-hal-matrix.c" "src/esp32-hal-spi.c"
                            "src/SPI.cpp"
                       INCLUDE_DIRS "src"
                       REQUIRES )