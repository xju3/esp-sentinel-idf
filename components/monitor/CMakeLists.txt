idf_component_register(SRCS "task_monitor.c"
                    INCLUDE_DIRS "."
                    REQUIRES icm42688p
                    PRIV_REQUIRES algo config logger app)
