add_executable(test_robot_model_hyrodyn test_robot_model_hyrodyn.cpp ../../suite.cpp ../test_robot_model.cpp)
target_link_libraries(test_robot_model_hyrodyn
                      wbc-robot_models-hyrodyn
                      ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})

