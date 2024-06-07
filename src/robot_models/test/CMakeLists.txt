add_executable(test_model_consistency test_model_consistency.cpp test_robot_model.cpp)
target_link_libraries(test_model_consistency
                     wbc-robot_models-hyrodyn
                     wbc-robot_models-rbdl
                     wbc-robot_models-pinocchio
                     Boost::unit_test_framework)
