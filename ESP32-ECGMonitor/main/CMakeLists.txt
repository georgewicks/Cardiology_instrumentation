
idf_component_register(SRCS "ECG_Testing.c" "main.c" "I2C_Handling.c" "ECG_WiFi.c" "logic_analyzer_sd_capture.c"
                       INCLUDE_DIRS "include" "."
                       PRIV_INCLUDE_DIRS "."
                       REQUIRES "spi_flash" "esp_wifi" "nvs_flash" "esp_event" "esp_timer" "driver" "log" "fatfs" "ads111x" "ds3231" "PCF8574" "QRSDetect" "i2c_eeprom" "Logging" "ECG-ISHNE" "MPU6050" "logic_analyzer"
                       PRIV_REQUIRES "spi_flash" "sdmmc")
