find_package(Boost COMPONENTS system filesystem unit_test_framework serialization REQUIRED)
add_executable(test_velocity_scene test_velocity_scene.cpp)
target_link_libraries(test_velocity_scene
                      wbc-scenes-velocity
                      wbc-robot_models-rbdl
                      wbc-solvers-hls
                      Boost::unit_test_framework)

