# Device layer: the peers of the gateway. Right now the STM32F103 sensor node on
# the UART, the ST7789 panel on the SPI bus and the rotary knob on the PCNT.
# esp_lcd and esp_driver_spi are public because dev_lcd.h hands the panel handles
# and the SPI host out, the rest is private to the implementations.
idf_component_register(SRCS "src/dev_stm32.c" "src/dev_lcd.c" "src/dev_encoder.c"
                       INCLUDE_DIRS "inc"
                       REQUIRES protocol esp_lcd esp_driver_spi esp_hal_gpspi
                       PRIV_REQUIRES bsp esp_driver_ledc esp_driver_gpio esp_driver_pcnt esp_timer)