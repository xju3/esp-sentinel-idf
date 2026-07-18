# The following five lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.16)

# Make nested components under components discoverable
set(EXTRA_COMPONENT_DIRS "components")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
# 告诉构建系统：除了 components/ 下的直接子目录，还要搜索 components/group1/ 下的子目录作为组件
# set(EXTRA_COMPONENT_DIRS "" )

# --------------------------------------------------------------------------
# LIS2DH12 hardware feature gate
# Pass -DLIS2=1 (default) to enable LIS2DH12 + WoM, or -DLIS2=0 to disable.
# --------------------------------------------------------------------------
if(NOT DEFINED LIS2)
    set(LIS2 1)
endif()

# VERSION is shared by the application descriptor and device_profile.json.
# It must be set before project() so ESP-IDF uses it as PROJECT_VER instead of
# falling back to `git describe`.
if(NOT DEFINED VERSION OR VERSION STREQUAL "")
    message(FATAL_ERROR "VERSION is required (example: -DVERSION=1.0.0)")
endif()
string(LENGTH "${VERSION}" VERSION_LENGTH)
if(VERSION_LENGTH GREATER 31 OR NOT VERSION MATCHES "^[A-Za-z0-9._-]+$")
    message(FATAL_ERROR "Invalid application version: ${VERSION}")
endif()
set(PROJECT_VER "${VERSION}")

# Component source lists are evaluated by project(), so the feature value must
# be defined before project() runs.
project(sentinel)

# Use idf_build_set_property so the definition propagates to all IDF components
idf_build_set_property(COMPILE_DEFINITIONS "-DLIS2=${LIS2}" APPEND)
message(STATUS "LIS2DH12 support: LIS2=${LIS2}")

# 在创建SPIFFS镜像前的预处理
set(RESOURCES_SYSTEM_DIR "${CMAKE_CURRENT_SOURCE_DIR}/resources/system")

# 如果www目录存在，清理不必要的字体格式，只保留.woff2
if(EXISTS "${RESOURCES_SYSTEM_DIR}/www")
    file(GLOB_RECURSE FONTS_TO_REMOVE "${RESOURCES_SYSTEM_DIR}/www/*.ttf" "${RESOURCES_SYSTEM_DIR}/www/*.woff")
    if(FONTS_TO_REMOVE)
        file(REMOVE ${FONTS_TO_REMOVE})
    endif()
endif()

# 创建临时目录用于SPIFFS
set(SPIFFS_TEMP_DIR "${CMAKE_CURRENT_BINARY_DIR}/spiffs_temp")
file(MAKE_DIRECTORY "${SPIFFS_TEMP_DIR}/w" "${SPIFFS_TEMP_DIR}/c")

# 复制资源文件并重命名字体文件为更短的名字
if(EXISTS "${RESOURCES_SYSTEM_DIR}/www")
    file(GLOB www_files "${RESOURCES_SYSTEM_DIR}/www/*")
    foreach(file ${www_files})
        get_filename_component(filename "${file}" NAME)
        set(dest_name "${filename}")
        
        # 缩短KaTeX字体文件名
        string(REPLACE "KaTeX_AMS-Regular" "A" dest_name "${dest_name}")
        string(REPLACE "KaTeX_Caligraphic-Bold" "CB" dest_name "${dest_name}")
        string(REPLACE "KaTeX_Caligraphic-Regular" "CR" dest_name "${dest_name}")
        string(REPLACE "KaTeX_Fraktur-Bold" "FB" dest_name "${dest_name}")
        string(REPLACE "KaTeX_Fraktur-Regular" "FR" dest_name "${dest_name}")
        string(REPLACE "KaTeX_Main-BoldItalic" "MBI" dest_name "${dest_name}")
        string(REPLACE "KaTeX_Main-Bold" "MB" dest_name "${dest_name}")
        string(REPLACE "KaTeX_Main-Italic" "MI" dest_name "${dest_name}")
        string(REPLACE "KaTeX_Main-Regular" "MR" dest_name "${dest_name}")
        string(REPLACE "KaTeX_Math-BoldItalic" "MBI" dest_name "${dest_name}")
        string(REPLACE "KaTeX_Math-Italic" "MaI" dest_name "${dest_name}")
        string(REPLACE "KaTeX_SansSerif-Bold" "SB" dest_name "${dest_name}")
        string(REPLACE "KaTeX_SansSerif-Italic" "SI" dest_name "${dest_name}")
        string(REPLACE "KaTeX_SansSerif-Regular" "SR" dest_name "${dest_name}")
        string(REPLACE "KaTeX_Script-Regular" "SC" dest_name "${dest_name}")
        string(REPLACE "KaTeX_Size1-Regular" "Z1" dest_name "${dest_name}")
        string(REPLACE "KaTeX_Size2-Regular" "Z2" dest_name "${dest_name}")
        string(REPLACE "KaTeX_Size3-Regular" "Z3" dest_name "${dest_name}")
        string(REPLACE "KaTeX_Size4-Regular" "Z4" dest_name "${dest_name}")
        string(REPLACE "KaTeX_Typewriter-Regular" "TT" dest_name "${dest_name}")
        
        configure_file("${file}" "${SPIFFS_TEMP_DIR}/w/${dest_name}" COPYONLY)
    endforeach()
endif()

if(EXISTS "${RESOURCES_SYSTEM_DIR}/config")
    file(GLOB config_files "${RESOURCES_SYSTEM_DIR}/config/*")
    file(COPY ${config_files} DESTINATION "${SPIFFS_TEMP_DIR}/c/")
endif()
# CMakeLists.txt 或 sdkconfig
# target_compile_definitions(${COMPONENT_TARGET} PRIVATE AT=1)

# 将 SPIFFS_TEMP_DIR 打包进 10MB 的 system 分区
spiffs_create_partition_image(system ${SPIFFS_TEMP_DIR} FLASH_IN_PROJECT)

# SN只用于生成设备专属的user.bin，绝不作为应用固件的编译宏。
if(NOT DEFINED SN OR SN STREQUAL "")
    message(FATAL_ERROR "SN is required to generate user.bin (example: -DSN=26SH00101)")
endif()

if(NOT DEFINED DEV)
    set(DEV 0)
endif()
add_compile_definitions(DEV_MODE=${DEV})
string(LENGTH "${SN}" SN_LENGTH)
if(SN_LENGTH GREATER 63 OR NOT SN MATCHES "^[A-Za-z0-9_-]+$")
    message(FATAL_ERROR "Invalid sensor SN: ${SN}")
endif()

string(TIMESTAMP DEVICE_PROFILE_GENERATED_AT "%Y-%m-%dT%H:%M:%SZ" UTC)

set(USER_SPIFFS_DIR "${CMAKE_CURRENT_BINARY_DIR}/user_spiffs")
file(REMOVE_RECURSE "${USER_SPIFFS_DIR}")
file(MAKE_DIRECTORY "${USER_SPIFFS_DIR}")
file(COPY "${CMAKE_CURRENT_SOURCE_DIR}/resources/user/"
     DESTINATION "${USER_SPIFFS_DIR}")
file(WRITE "${USER_SPIFFS_DIR}/device_profile.json"
     "{\n"
     "  \"sn\": \"${SN}\",\n"
     "  \"version\": \"${VERSION}\",\n"
     "  \"generated_at\": \"${DEVICE_PROFILE_GENERATED_AT}\",\n"
     "  \"device_id\": \"\"\n"
     "}\n")

message(STATUS "Generating user.bin device profile: SN=${SN}, VERSION=${VERSION}, generated_at=${DEVICE_PROFILE_GENERATED_AT}")
spiffs_create_partition_image(user ${USER_SPIFFS_DIR} FLASH_IN_PROJECT)
