add_executable(test_acceleration_scene_tsid test_acceleration_scene_tsid.cpp)
target_link_libraries(test_acceleration_scene_tsid
                      wbc-scenes-acceleration_tsid
                      wbc-robot_models-pinocchio
                      wbc-solvers-qpoases
                      Boost::unit_test_framework)

add_test(NAME test_acceleration_scene_tsid COMMAND test_acceleration_scene_tsid)
