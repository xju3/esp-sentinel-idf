idf_component_register(SRCS task_baseline.c task_monitor.c task_fft.c
                       INCLUDE_DIRS "include"
                       REQUIRES esp-dsp logger icm42688p dispatcher algo)

if(DEFINED DEVICE_ID)
    target_compile_definitions(${COMPONENT_LIB} PRIVATE DEVICE_ID="${DEVICE_ID}")
endif()
