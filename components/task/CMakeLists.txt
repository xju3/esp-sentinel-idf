idf_component_register(
    SRCS 
        "src/task_daq.c" 
        "src/task_rms.c" 
        "src/task_fft.c"
        "src/task_diag_fusion.c"
        "src/task_kurtosis.c"
        "src/task_envelope.c"
        "src/daq_worker.c" 
        "src/task_baseline.c" 
        "src/wom_lis2dh12.c"
        "src/off_sleep_manager.c"
        "src/task_stash.c"
    INCLUDE_DIRS 
        "include" 
    REQUIRES 
        esp-dsp driver peri net util algo msg)

if(DEFINED DEVICE_ID)
    target_compile_definitions(${COMPONENT_LIB} PRIVATE DEVICE_ID="${DEVICE_ID}")
endif()

if(DEFINED SN)
    target_compile_definitions(${COMPONENT_LIB} PRIVATE SN=${SN})
endif()


# target_sources(${COMPONENT_LIB} PRIVATE)
