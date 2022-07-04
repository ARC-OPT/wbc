add_subdirectory(kdl)
if(USE_HYRODYN)
    add_subdirectory(hyrodyn)
endif()
if(USE_PINOCCHIO)
    add_subdirectory(pinocchio)
endif()

