add_executable(test_acceleration_scene test_acceleration_scene.cpp)
target_link_libraries(test_acceleration_scene
                      wbc-scenes-acceleration
                      wbc-robot_models-pinocchio
                      wbc-solvers-qpoases
                      Boost::unit_test_framework)
