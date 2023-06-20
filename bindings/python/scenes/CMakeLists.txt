add_subdirectory(velocity)
add_subdirectory(velocity_qp)
add_subdirectory(acceleration)
add_subdirectory(acceleration_tsid)
add_subdirectory(acceleration_reduced_tsid)

install(FILES __init__.py
        DESTINATION ${PYTHON_INSTALL_PATH}/wbc/scenes)
