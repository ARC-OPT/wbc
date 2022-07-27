find_package(Boost COMPONENTS system filesystem unit_test_framework REQUIRED)
include_directories(${PROJECT_SOURCE_DIR}/src)

add_executable(test_velocity_scene test_velocity_scene.cpp ../suite.cpp)
target_link_libraries(test_velocity_scene
                      wbc-scenes
                      wbc-robot_models-kdl
                      wbc-tools
                      wbc-solvers-hls
                      ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})

add_executable(test_velocity_scene_quadratic_cost test_velocity_scene_quadratic_cost.cpp ../suite.cpp)
target_link_libraries(test_velocity_scene_quadratic_cost
                      wbc-scenes
                      wbc-robot_models-kdl
                      wbc-solvers-qpoases
                      ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})

add_executable(test_acceleration_scene test_acceleration_scene.cpp ../suite.cpp)
target_link_libraries(test_acceleration_scene
                      wbc-scenes
                      wbc-robot_models-kdl
                      wbc-tools
                      wbc-solvers-qpoases
                      ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})

