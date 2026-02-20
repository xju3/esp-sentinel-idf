# The following five lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.16)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(sentinel)
# （下面是你之前加的打包 www 文件夹的代码）
spiffs_create_partition_image(storage www FLASH_IN_PROJECT)