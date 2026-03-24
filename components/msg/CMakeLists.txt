idf_component_register(
    SRCS 
        "src/msg_machine_state.pb-c.c" 
    INCLUDE_DIRS 
        "include"
    REQUIRES 
        protobuf-c)

if(DEFINED DEVICE_ID)
    target_compile_definitions(${COMPONENT_LIB} PRIVATE DEVICE_ID="${DEVICE_ID}")
endif()
