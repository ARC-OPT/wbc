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
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/src ${base-types_INCLUDE_DIRS} )
link_directories(${base-types_LIBRARY_DIRS})

PYTHON_ADD_MODULE(controllers controllers.cpp)

# Set up the libraries and header search paths for this target
target_link_libraries(controllers ${Boost_LIBRARIES} ${PYTHON_LIBRARIES} wbc-controllers)
target_include_directories(controllers PRIVATE ${PYTHON_INCLUDE_DIRS} )

install(TARGETS controllers
        DESTINATION ${PYTHON_INSTALL_PATH}/wbc)
