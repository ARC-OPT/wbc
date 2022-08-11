pkg_search_module(rbdl REQUIRED IMPORTED_TARGET rbdl)
pkg_search_module(rbdl-urdfreader REQUIRED IMPORTED_TARGET rbdl-urdfreader)

target_link_libraries(test_robot_model_hyrodyn
                      wbc-robot_models-hyrodyn
                      wbc-tools
                      ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})

