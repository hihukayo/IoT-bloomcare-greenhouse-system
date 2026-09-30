idf_component_register(SRCS "main.c"
                            "tasks/src/task_comm.c"
                            "tasks/src/task_sensor.c"
                            "tasks/src/task_gateway.c"
                            "tasks/src/task_display.c"
                            "wifi/src/task_net.c"
                       INCLUDE_DIRS "." "tasks/inc" "wifi/inc" "wifi"
                       PRIV_REQUIRES protocol device bsp lvgl esp_lvgl_port
                                      esp_wifi esp_netif nvs_flash esp_event)
