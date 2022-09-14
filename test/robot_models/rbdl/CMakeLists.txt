add_executable(test_robot_model_rbdl test_robot_model_rbdl.cpp ../../suite.cpp ../test_robot_model.cpp)
target_link_libraries(test_robot_model_rbdl
                      wbc-robot_models-rbdl
                      Boost::unit_test_framework)

