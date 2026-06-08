idf_component_register(
    SRCS 
        "src/task_daq.c" 
        "src/task_rms.c" 
        "src/task_fft.c"
        "src/task_diag_fusion.c"
        "src/task_kurtosis.c"
        "src/task_envelope.c"
        "src/daq_worker.c" 
        "src/report_pipeline.c"
        "src/wom_lis2dh12.c"
        "src/off_sleep_manager.c"
        "src/task_stash.c"
        "src/task_http_message.c"
        "src/task_ota.c"
    INCLUDE_DIRS 
        "include" 
    REQUIRES 
        esp-dsp driver bsp peri net util algo msg json esp_http_client esp_timer)

if(DEFINED SN)
    target_compile_definitions(${COMPONENT_LIB} PRIVATE REPORT_SN="${SN}")
endif()

if(DEFINED POINTS)
    target_compile_definitions(${COMPONENT_LIB} PRIVATE POINTS=${POINTS})
endif()

if(DEFINED RANGE)
    target_compile_definitions(${COMPONENT_LIB} PRIVATE RANGE=${RANGE})
endif()

# target_sources(${COMPONENT_LIB} PRIVATE)
