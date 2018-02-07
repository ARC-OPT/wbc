find_package(Boost COMPONENTS system filesystem unit_test_framework REQUIRED)
include_directories(${PROJECT_SOURCE_DIR}/src)

add_subdirectory(core)
add_subdirectory(robot_models)
add_subdirectory(scenes)
add_subdirectory(solvers)
add_subdirectory(tools)
