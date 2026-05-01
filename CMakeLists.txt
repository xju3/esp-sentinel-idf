# The following five lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.16)

# Make nested components under components discoverable
set(EXTRA_COMPONENT_DIRS "components")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
# 告诉构建系统：除了 components/ 下的直接子目录，还要搜索 components/group1/ 下的子目录作为组件
# set(EXTRA_COMPONENT_DIRS "" )
project(sentinel)

# 在创建SPIFFS镜像前的预处理
set(RESOURCES_SYSTEM_DIR "${CMAKE_CURRENT_SOURCE_DIR}/resources/system")

# 创建临时目录用于SPIFFS
set(SPIFFS_TEMP_DIR "${CMAKE_CURRENT_BINARY_DIR}/spiffs_temp")
file(MAKE_DIRECTORY "${SPIFFS_TEMP_DIR}")

set(SYSTEM_SPIFFS_PREP_CMDS
    COMMAND ${CMAKE_COMMAND} -E rm -rf "${SPIFFS_TEMP_DIR}/w"
    COMMAND ${CMAKE_COMMAND} -E rm -rf "${SPIFFS_TEMP_DIR}/c"
    COMMAND ${CMAKE_COMMAND} -E make_directory "${SPIFFS_TEMP_DIR}/w"
    COMMAND ${CMAKE_COMMAND} -E make_directory "${SPIFFS_TEMP_DIR}/c"
)

set(SYSTEM_SPIFFS_DEPENDS)

# 如果www目录存在，清理不必要的字体格式，只保留.woff2
if(EXISTS "${RESOURCES_SYSTEM_DIR}/www")
    file(GLOB_RECURSE FONTS_TO_REMOVE CONFIGURE_DEPENDS
        "${RESOURCES_SYSTEM_DIR}/www/*.ttf"
        "${RESOURCES_SYSTEM_DIR}/www/*.woff")
    if(FONTS_TO_REMOVE)
        file(REMOVE ${FONTS_TO_REMOVE})
    endif()

    file(GLOB WWW_FILES CONFIGURE_DEPENDS "${RESOURCES_SYSTEM_DIR}/www/*")
    list(APPEND SYSTEM_SPIFFS_DEPENDS ${WWW_FILES})
    foreach(file ${WWW_FILES})
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

        list(APPEND SYSTEM_SPIFFS_PREP_CMDS
            COMMAND ${CMAKE_COMMAND} -E copy_if_different "${file}" "${SPIFFS_TEMP_DIR}/w/${dest_name}")
    endforeach()
endif()

if(EXISTS "${RESOURCES_SYSTEM_DIR}/config")
    file(GLOB CONFIG_FILES CONFIGURE_DEPENDS "${RESOURCES_SYSTEM_DIR}/config/*")
    list(APPEND SYSTEM_SPIFFS_DEPENDS ${CONFIG_FILES})
    foreach(file ${CONFIG_FILES})
        list(APPEND SYSTEM_SPIFFS_PREP_CMDS
            COMMAND ${CMAKE_COMMAND} -E copy_if_different "${file}" "${SPIFFS_TEMP_DIR}/c/")
    endforeach()
endif()

add_custom_target(prepare_system_spiffs ALL
    ${SYSTEM_SPIFFS_PREP_CMDS}
    DEPENDS ${SYSTEM_SPIFFS_DEPENDS}
    COMMENT "Syncing system resources into SPIFFS staging directory")

# 将 SPIFFS_TEMP_DIR 打包进 10MB 的 system 分区
spiffs_create_partition_image(system ${SPIFFS_TEMP_DIR} FLASH_IN_PROJECT DEPENDS prepare_system_spiffs)

# 将 resources/user 目录打包进 64KB 的 user 分区 (初始的用户配置文件)
spiffs_create_partition_image(user resources/user FLASH_IN_PROJECT)
