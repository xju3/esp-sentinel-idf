idf_component_register(
    SRCS 
        "src/task_daq.c" 
        "src/task_rms.c" 
        "src/task_fft.c"
        "src/task_kurtosis.c"
        "src/daq_worker.c" 
        "src/task_baseline.c" 
        "src/wom_lis2dh12.c" 
    INCLUDE_DIRS 
        "include" 
    REQUIRES 
        esp-dsp driver peri net util algo)

if(DEFINED DEVICE_ID)
    target_compile_definitions(${COMPONENT_LIB} PRIVATE DEVICE_ID="${DEVICE_ID}")
endif()