add_executable(test_velocity_scene test_velocity_scene.cpp)
target_link_libraries(test_velocity_scene
                      wbc-scenes-velocity
                      wbc-robot_models-pinocchio
                      wbc-solvers-hls
                      Boost::unit_test_framework)

add_test(NAME test_velocity_scene COMMAND test_velocity_scene)

