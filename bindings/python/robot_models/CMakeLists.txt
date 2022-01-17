add_subdirectory(kdl)
if(USE_HYRODYN)
    add_subdirectory(hyrodyn)
endif()

find_package(PythonInterp REQUIRED)
set(PYTHON_INSTALL_PATH ${CMAKE_INSTALL_PREFIX}/lib/python${PYTHON_VERSION_MAJOR}.${PYTHON_VERSION_MINOR}/site-packages)
install(FILES __init__.py
        DESTINATION ${PYTHON_INSTALL_PATH}/wbc/robot_models)
