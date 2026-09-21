# Device layer: the peer devices behind the gateway, currently the STM32F103
# sensor node. Later an OLED or another sensor belongs in here as well.
idf_component_register(SRCS "src/dev_stm32.c"
                       INCLUDE_DIRS "inc"
                       REQUIRES protocol
                       PRIV_REQUIRES bsp)
