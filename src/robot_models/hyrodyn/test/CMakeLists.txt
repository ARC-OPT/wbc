find_package(Boost COMPONENTS system filesystem unit_test_framework serialization REQUIRED)
add_executable(test_robot_model_hyrodyn test_robot_model_hyrodyn.cpp ../../test/test_robot_model.cpp)
target_link_libraries(test_robot_model_hyrodyn
                      wbc-robot_models-hyrodyn
                      Boost::unit_test_framework)
