idf_component_register(SRCS "task_monitor.c"
                       INCLUDE_DIRS "."
                       REQUIRES freertos config logger icm42688p algo)
