idf_component_register(
    SRCS 
        "src/task_daq.c" 
        "src/daq_worker.c" 
        "src/task_rms.c" 
        "src/task_fft.c"
        "src/wom_lis2dh20.c" 
        "src/wom_icm_42688.c" 
    INCLUDE_DIRS 
        "include" 
    REQUIRES 
        esp-dsp driver peri net util algo)

if(DEFINED DEVICE_ID)
    target_compile_definitions(${COMPONENT_LIB} PRIVATE DEVICE_ID="${DEVICE_ID}")
endif()