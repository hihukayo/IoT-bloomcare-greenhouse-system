idf_component_register(SRCS "main.c"
                            "tasks/src/task_comm.c"
                            "tasks/src/task_sensor.c"
                            "tasks/src/task_gateway.c"
                       INCLUDE_DIRS "." "tasks/inc"
                       PRIV_REQUIRES protocol device bsp)
