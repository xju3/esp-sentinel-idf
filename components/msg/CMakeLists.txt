idf_component_register(
    SRCS 
        "src/msg_sentinel.pb-c.c" "src/data_dispatcher.c"
    INCLUDE_DIRS 
        "include"
    REQUIRES 
        protobuf-c net)

if(DEFINED DEVICE_ID)
    target_compile_definitions(${COMPONENT_LIB} PRIVATE DEVICE_ID="${DEVICE_ID}")
endif()

if(DEFINED SN)
    target_compile_definitions(${COMPONENT_LIB} PRIVATE SN=${SN})
endif()
