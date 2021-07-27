cmake_minimum_required (VERSION 2.8.11)
project(wbc)

include(FindPkgConfig)
add_compile_options(-std=c++11)

set(PROJECT_VERSION 0.2)
set(API_VERSION ${PROJECT_VERSION})

include_directories(${PROJECT_SOURCE_DIR}/src)
list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

add_subdirectory(src)
add_subdirectory(test)
add_subdirectory(examples)
add_subdirectory(bin)
if(USE_PYTHON)
    add_subdirectory(python)
endif()
