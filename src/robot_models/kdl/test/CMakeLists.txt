find_package(Boost COMPONENTS system filesystem unit_test_framework serialization REQUIRED)
add_executable(test_robot_model_kdl test_robot_model_kdl.cpp ../../test/test_robot_model.cpp)
target_link_libraries(test_robot_model_kdl
                      wbc-robot_models-kdl
                      Boost::unit_test_framework)
