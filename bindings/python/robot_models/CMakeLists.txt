
add_subdirectory(rbdl)
if(USE_HYRODYN)
    add_subdirectory(hyrodyn)
endif()
if(USE_KDL)
    add_subdirectory(kdl)
endif()
if(USE_PINOCCHIO)
    add_subdirectory(pinocchio)
endif()

install(FILES __init__.py
        DESTINATION ${PYTHON_INSTALL_PATH}/wbc/robot_models)
