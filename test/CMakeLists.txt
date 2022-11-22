find_package(Boost COMPONENTS system filesystem unit_test_framework serialization REQUIRED)

add_subdirectory(controllers)
add_subdirectory(core)
add_subdirectory(robot_models)
add_subdirectory(scenes)
add_subdirectory(solvers)
