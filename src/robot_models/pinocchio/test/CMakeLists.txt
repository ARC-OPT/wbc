add_executable(test_robot_model_pinocchio test_robot_model_pinocchio.cpp ../../test/test_robot_model.cpp)
target_link_libraries(test_robot_model_pinocchio
                      wbc-robot_models-pinocchio
                      Boost::unit_test_framework
                      Boost::system
                      Boost::filesystem
                      Boost::serialization)
