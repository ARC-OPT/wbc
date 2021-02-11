cmake_minimum_required(VERSION 2.8.11)

find_package(PythonInterp REQUIRED)
find_package(PythonLibs REQUIRED)

set(PYTHON_INSTALL_PATH ${CMAKE_INSTALL_PREFIX}/lib/python${PYTHON_VERSION_MAJOR}.${PYTHON_VERSION_MINOR}/site-packages)

if (UNIX AND NOT APPLE)
    if (PYTHON_VERSION_MAJOR EQUAL 3)
        find_package(Boost COMPONENTS python${PYTHON_VERSION_MAJOR} numpy${PYTHON_VERSION_MAJOR})
    else()
        find_package(Boost COMPONENTS python numpy${PYTHON_VERSION_MAJOR})
    endif()
else()
    if (PYTHON_VERSION_MAJOR EQUAL 3)
        find_package(Boost COMPONENTS python${PYTHON_VERSION_MAJOR}${PYTHON_VERSION_MINOR} numpy${PYTHON_VERSION_MAJOR})
    else()
        find_package(Boost COMPONENTS python${PYTHON_VERSION_MAJOR}${PYTHON_VERSION_MINOR} numpy${PYTHON_VERSION_MAJOR})
    endif()
endif()

message(STATUS "PYTHON_VERSION_MAJOR.PYTHON_VERSION_MINOR = ${PYTHON_VERSION_MAJOR}.${PYTHON_VERSION_MINOR}")
message(STATUS "PYTHON_LIBRARIES = ${PYTHON_LIBRARIES}")
message(STATUS "PYTHON_EXECUTABLE = ${PYTHON_EXECUTABLE}")
message(STATUS "PYTHON_INCLUDE_DIRS = ${PYTHON_INCLUDE_DIRS}")
message(STATUS "Boost_LIBRARIES = ${Boost_LIBRARIES}")
message(STATUS "PYTHON_INSTALL_PATH = ${PYTHON_INSTALL_PATH}")

pkg_search_module(base-types REQUIRED base-types)
pkg_search_module(hyrodyn REQUIRED hyrodyn)
pkg_search_module(orocos-kdl REQUIRED orocos-kdl)
pkg_search_module(kdl_parser REQUIRED kdl_parser)
include_directories(${base-types_INCLUDE_DIRS} ${hyrodyn_INCLUDE_DIRS} ${orocos-kdl_INCLUDE_DIRS} ${kdl_parser_INCLUDE_DIRS})
link_directories(${base-types_LIBRARY_DIRS} ${hyrodyn_LIBRARY_DIRS} ${orocos-kd_LIBRARY_DIRS} ${kdl_parser_LIBRARY_DIRS})

# Add a shared module - modules are intended to be imported at runtime.
# - This is where you add the source files
PYTHON_ADD_MODULE(core core.cpp)
PYTHON_ADD_MODULE(robot_models robot_models.cpp)

# Set up the libraries and header search paths for this target
target_link_libraries(core ${Boost_LIBRARIES} ${PYTHON_LIBRARIES} wbc-core)
target_include_directories(core PRIVATE ${PYTHON_INCLUDE_DIRS})
target_link_libraries(robot_models ${Boost_LIBRARIES} ${PYTHON_LIBRARIES} wbc-robot_models)
target_include_directories(robot_models PRIVATE ${PYTHON_INCLUDE_DIRS} )
install(TARGETS core
        DESTINATION ${PYTHON_INSTALL_PATH}/wbc)
install(TARGETS robot_models
        DESTINATION ${PYTHON_INSTALL_PATH}/wbc)
install(FILES __init__.py
        DESTINATION ${PYTHON_INSTALL_PATH}/wbc)
