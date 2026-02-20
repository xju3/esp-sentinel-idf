# The following five lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.16)

# Make nested components under components/sensor discoverable
set(EXTRA_COMPONENT_DIRS "${CMAKE_CURRENT_LIST_DIR}/components/sensor")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(sentinel)

# 将 resources/system 目录打包进 10MB 的 system 分区 (存放 www 和 config)
spiffs_create_partition_image(system resources/system FLASH_IN_PROJECT)

# 将 resources/user 目录打包进 64KB 的 user 分区 (初始的用户配置文件)
spiffs_create_partition_image(user resources/user FLASH_IN_PROJECT)
