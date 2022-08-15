add_executable(test_robot_model_kdl test_robot_model_kdl.cpp ../../suite.cpp ../test_robot_model.cpp)
target_link_libraries(test_robot_model_kdl
                      wbc-robot_models-kdl
                      ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})

