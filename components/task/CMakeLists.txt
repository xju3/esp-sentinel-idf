idf_component_register(SRCS src/task_baseline.c src/task_monitor.c src/task_fft.c
                       INCLUDE_DIRS "include"
                       REQUIRES esp-dsp peri net util algo)

if(DEFINED DEVICE_ID)
    target_compile_definitions(${COMPONENT_LIB} PRIVATE DEVICE_ID="${DEVICE_ID}")
endif()
