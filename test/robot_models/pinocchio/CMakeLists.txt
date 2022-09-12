add_executable(test_robot_model_pinocchio test_robot_model_pinocchio.cpp ../../suite.cpp ../test_robot_model.cpp)
target_link_libraries(test_robot_model_pinocchio
                      wbc-robot_models-pinocchio
                      ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})
