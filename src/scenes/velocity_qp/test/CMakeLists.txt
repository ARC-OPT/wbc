add_executable(test_velocity_scene_quadratic_cost test_velocity_scene_quadratic_cost.cpp)
target_link_libraries(test_velocity_scene_quadratic_cost
                      wbc-scenes-velocity_qp
                      wbc-robot_models-pinocchio
                      wbc-solvers-qpoases
                      Boost::unit_test_framework)

add_test(NAME test_velocity_scene_quadratic_cost COMMAND test_velocity_scene_quadratic_cost)

