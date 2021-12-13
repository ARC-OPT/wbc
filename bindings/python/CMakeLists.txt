add_subdirectory(core)
add_subdirectory(controllers)
add_subdirectory(robot_models)
add_subdirectory(scenes)
add_subdirectory(solvers)

set(PYTHON_INSTALL_PATH ${CMAKE_INSTALL_PREFIX}/lib/python${PYTHON_VERSION_MAJOR}.${PYTHON_VERSION_MINOR}/site-packages)
install(FILES __init__.py
        DESTINATION ${PYTHON_INSTALL_PATH}/wbc)
