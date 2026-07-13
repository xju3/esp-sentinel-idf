set(TASK_SRCS
    "src/task_daq.c"
    "src/report_pipeline.c"
    "src/server_report_task_scheduler.c"
    "src/task_http_message.c"
    "src/task_ota.c"
    "src/task_status_report.c"
)

# wom_lis2dh12.c and off_sleep_manager.c are only compiled when LIS2DH12 is present (-DLIS2=1)
if(LIS2)
    list(APPEND TASK_SRCS "src/wom_lis2dh12.c" "src/off_sleep_manager.c")
endif()

idf_component_register(
    SRCS ${TASK_SRCS}
    INCLUDE_DIRS 
        "include" 
    REQUIRES 
        esp-dsp driver esp_driver_gpio bsp peri net util algo msg json esp_http_client esp_timer esp_hw_support)


if(DEFINED SN)
    target_compile_definitions(${COMPONENT_LIB} PRIVATE REPORT_SN="${SN}")
endif()





# target_sources(${COMPONENT_LIB} PRIVATE)
