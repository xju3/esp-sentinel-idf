idf_component_register(
    SRCS 
        "src/task_daq.c" 
        "src/report_pipeline.c"
        "src/report_upload_cache.c"
        "src/server_report_task_scheduler.c"
        "src/task_http_message.c"
        "src/task_ota.c"
        "src/task_status_report.c"
    INCLUDE_DIRS 
        "include" 
    REQUIRES 
        esp-dsp driver esp_driver_gpio bsp peri net util algo msg json esp_http_client esp_timer)

if(DEFINED SN)
    target_compile_definitions(${COMPONENT_LIB} PRIVATE REPORT_SN="${SN}")
endif()





# target_sources(${COMPONENT_LIB} PRIVATE)
