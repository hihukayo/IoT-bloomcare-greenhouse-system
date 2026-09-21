# Protocol layer: pure software, no ESP-IDF driver and no hardware reference.
# The very same frame format runs on the STM32 node, see Components/comp_link.
idf_component_register(SRCS "src/link_protocol.c"
                       INCLUDE_DIRS "inc")
