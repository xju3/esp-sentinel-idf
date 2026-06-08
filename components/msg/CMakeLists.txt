idf_component_register(
    SRCS 
        "src/msg_sentinel.pb-c.c" "src/data_dispatcher.c"
    INCLUDE_DIRS 
        "include"
    REQUIRES 
        protobuf-c net)