add_executable(test_acceleration_scene_reduced_tsid test_acceleration_scene_reduced_tsid.cpp)
target_link_libraries(test_acceleration_scene_reduced_tsid
                      wbc-scenes-acceleration_reduced_tsid
                      wbc-robot_models-pinocchio
                      wbc-solvers-qpoases
                      Boost::unit_test_framework)

add_test(NAME test_acceleration_scene_reduced_tsid COMMAND test_acceleration_scene_reduced_tsid)
