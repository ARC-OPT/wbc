find_package(Boost COMPONENTS system filesystem REQUIRED)

add_subdirectory(robot_models)
add_subdirectory(scenes)
if(USE_QPSWIFT AND USE_EIQUADPROG AND USE_PROXQP)
    add_subdirectory(solvers)
endif()
