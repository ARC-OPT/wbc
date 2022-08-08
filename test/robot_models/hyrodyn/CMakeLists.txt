pkg_search_module(base-types REQUIRED base-types)
pkg_search_module(hyrodyn REQUIRED hyrodyn)
pkg_search_module(rbdl REQUIRED rbdl)

include_directories(${base-types_INCLUDE_DIRS}
                    ${hyrodyn_INCLUDE_DIRS}
                    ${rbdl_INCLUDE_DIRS})
link_directories(${base-types_LIBRARY_DIRS}
                 ${hyrodyn_LIBRARY_DIRS}
                 ${rbdl_LIBRARY_DIRS})


add_executable(test_robot_model_hyrodyn test_robot_model_hyrodyn.cpp ../../suite.cpp ../test_robot_model.cpp)
target_link_libraries(test_robot_model_hyrodyn
                      wbc-robot_models-hyrodyn
                      wbc-tools
                      ${rbdl_LIBRARIES}
                      ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})

