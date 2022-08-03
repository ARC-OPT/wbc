add_subdirectory(kdl)
if(USE_HYRODYN)
    add_subdirectory(hyrodyn)
endif()

install(FILES __init__.py
        DESTINATION ${PYTHON_INSTALL_PATH}/wbc/robot_models)
