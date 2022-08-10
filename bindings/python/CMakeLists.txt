find_package(PythonInterp 3 REQUIRED)
find_package(PythonLibs 3 REQUIRED)
find_package(Boost COMPONENTS python3 numpy3)

set(PYTHON_INSTALL_PATH ${CMAKE_INSTALL_PREFIX}/lib/python${PYTHON_VERSION_MAJOR}.${PYTHON_VERSION_MINOR}/site-packages)

message(STATUS "PYTHON_VERSION_MAJOR.PYTHON_VERSION_MINOR = ${PYTHON_VERSION_MAJOR}.${PYTHON_VERSION_MINOR}")
message(STATUS "PYTHON_LIBRARIES = ${PYTHON_LIBRARIES}")
message(STATUS "PYTHON_EXECUTABLE = ${PYTHON_EXECUTABLE}")
message(STATUS "PYTHON_INCLUDE_DIRS = ${PYTHON_INCLUDE_DIRS}")
message(STATUS "Boost_LIBRARIES = ${Boost_LIBRARIES}")
message(STATUS "PYTHON_INSTALL_PATH = ${PYTHON_INSTALL_PATH}")

add_subdirectory(core)
add_subdirectory(controllers)
add_subdirectory(robot_models)
add_subdirectory(scenes)
add_subdirectory(solvers)

install(FILES __init__.py
        DESTINATION ${PYTHON_INSTALL_PATH}/wbc)
