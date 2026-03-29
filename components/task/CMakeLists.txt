idf_component_register(
    SRCS 
        "src/task_daq.c" 
        "src/task_rms.c" 
        "src/task_fft.c"
        "src/task_kurtosis.c"
        "src/task_envelope.c"
        "src/daq_worker.c" 
        "src/task_baseline.c" 
        "src/speed_calibration.c" 
        "src/wom_lis2dh12.c"
        "src/task_state_machine.c"
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


if(DEFINED IMU)
    # IMU is used in preprocessor numeric comparisons (e.g., #if IMU == 1),
    # so ensure it is defined as a numeric literal, not a string.
    target_compile_definitions(${COMPONENT_LIB} PRIVATE IMU=${IMU})
endif()


# target_sources(${COMPONENT_LIB} PRIVATE)
