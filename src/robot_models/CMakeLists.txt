add_subdirectory(pinocchio)
if(ROBOT_MODEL_RBDL)
    add_subdirectory(rbdl)
endif()
if(ROBOT_MODEL_HYRODYN)
    add_subdirectory(hyrodyn)
endif()
