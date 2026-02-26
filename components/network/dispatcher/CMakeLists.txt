idf_component_register(SRCS "data_dispatcher.c"
                       INCLUDE_DIRS "."
                       REQUIRES freertos config logger mqtt
                       PRIV_REQUIRES app)
