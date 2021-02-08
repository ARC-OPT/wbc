cmake_minimum_required(VERSION 2.8.11)

set(PYTHON_INCLUDE_DIR /usr/include/python3.6)

# Find python and Boost - both are required dependencies
find_package(PythonLibs 3.6 REQUIRED)
find_package(Boost COMPONENTS python3 numpy3 REQUIRED)

pkg_search_module(base-types REQUIRED base-types)
pkg_search_module(hyrodyn REQUIRED hyrodyn)
include_directories(${base-types_INCLUDE_DIRS} ${hyrodyn_INCLUDE_DIRS})
link_directories(${base-types_LIBRARY_DIRS} ${hyrodyn_LIBRARY_DIRS})

# Without this, any build libraries automatically have names "lib{x}.so"
set(CMAKE_SHARED_MODULE_PREFIX "")

# Add a shared module - modules are intended to be imported at runtime.
# - This is where you add the source files
add_library(wbc_py MODULE wbc_py.cpp)

# Set up the libraries and header search paths for this target
target_link_libraries(wbc_py ${Boost_LIBRARIES} ${PYTHON_LIBRARIES} wbc-core)
target_include_directories(wbc_py PRIVATE ${PYTHON_INCLUDE_DIRS})
