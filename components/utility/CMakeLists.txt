idf_component_register(
    SRCS "src/config_manager.c" "src/logger.c" "src/fs_utils.c"
    INCLUDE_DIRS "include"
    REQUIRES spiffs json
)
