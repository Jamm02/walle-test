set(srcs "src/i2cdev.c" "src/adc.c" "src/bar_graph.c" "src/switches.c" "src/lsa.c" "src/motor_driver.c" "src/mpu6050.c" "src/servo.c" "src/utils.c")
set(include_dirs include)

idf_component_register(SRCS "${srcs}"
                       INCLUDE_DIRS "${include_dirs}"
                       REQUIRES driver efuse esp32 esp_adc_cal esp_common esp_event 
                                esp_rom freertos heap log soc spiffs)