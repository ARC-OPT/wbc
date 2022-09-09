pkg_search_module(rbdl REQUIRED IMPORTED_TARGET rbdl)
pkg_search_module(rbdl-urdfreader REQUIRED IMPORTED_TARGET rbdl-urdfreader)

add_executable(test_robot_model_hyrodyn test_robot_model_hyrodyn.cpp ../../suite.cpp ../test_robot_model.cpp)
target_link_libraries(test_robot_model_hyrodyn
                      wbc-robot_models-hyrodyn
                      wbc-tools
                      ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})

