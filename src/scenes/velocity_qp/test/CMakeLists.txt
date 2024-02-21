find_package(Boost COMPONENTS system filesystem unit_test_framework serialization REQUIRED)
add_executable(test_velocity_scene_quadratic_cost test_velocity_scene_quadratic_cost.cpp)
target_link_libraries(test_velocity_scene_quadratic_cost
                      wbc-scenes-velocity_qp
                      wbc-robot_models-rbdl
                      wbc-solvers-qpoases
                      Boost::unit_test_framework)

