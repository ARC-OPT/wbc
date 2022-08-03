add_subdirectory(qpoases)
add_subdirectory(hls)

install(FILES __init__.py
        DESTINATION ${PYTHON_INSTALL_PATH}/wbc/solvers)
