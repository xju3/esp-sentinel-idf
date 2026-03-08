idf_component_register(
    SRCS 
        "src/test_lis2dh12.c" 
    INCLUDE_DIRS 
        "include" 
    REQUIRES 
        esp-dsp driver peri net util algo task)

if(DEFINED DEVICE_ID)
    target_compile_definitions(${COMPONENT_LIB} PRIVATE DEVICE_ID="${DEVICE_ID}")
endif()