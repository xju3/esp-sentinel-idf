idf_component_register(
    SRCS "src/config_manager.c" "src/logger.c" "src/fs_utils.c" "src/iso_check.c" "src/machine_state.c" "src/startup_gate.c"
    INCLUDE_DIRS "include"
    REQUIRES spiffs json
)
