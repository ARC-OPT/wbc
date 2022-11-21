add_executable(test_velocity_scene test_velocity_scene.cpp ../suite.cpp)
target_link_libraries(test_velocity_scene
                      wbc-scenes
                      wbc-robot_models-pinocchio
                      wbc-solvers-hls
                      Boost::unit_test_framework)

add_executable(test_velocity_scene_quadratic_cost test_velocity_scene_quadratic_cost.cpp ../suite.cpp)
target_link_libraries(test_velocity_scene_quadratic_cost
                      wbc-scenes
                      wbc-robot_models-pinocchio
                      wbc-solvers-qpoases
                      Boost::unit_test_framework)

add_executable(test_acceleration_scene test_acceleration_scene.cpp ../suite.cpp)
target_link_libraries(test_acceleration_scene
                      wbc-scenes
                      wbc-robot_models-pinocchio
                      wbc-solvers-qpoases
                      Boost::unit_test_framework)

add_executable(test_acceleration_scene_tsid test_acceleration_scene_tsid.cpp ../suite.cpp)
target_link_libraries(test_acceleration_scene_tsid
                      wbc-scenes
                      wbc-robot_models-pinocchio
                      wbc-solvers-qpoases
                      Boost::unit_test_framework)

add_executable(test_acceleration_scene_reduced_tsid test_acceleration_scene_reduced_tsid.cpp ../suite.cpp)
target_link_libraries(test_acceleration_scene_reduced_tsid
                      wbc-scenes
                      wbc-robot_models-pinocchio
                      wbc-solvers-qpoases
                      Boost::unit_test_framework)


