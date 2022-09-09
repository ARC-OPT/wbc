pkg_search_module(base-types REQUIRED base-types)
pkg_search_module(pinocchio REQUIRED pinocchio)

include_directories(${base-types_INCLUDE_DIRS} ${pinocchio_INCLUDE_DIRS})
link_directories(${base-types_LIBRARY_DIRS} ${pinocchio_LIBRARY_DIRS})

add_executable(test_robot_model_pinocchio test_robot_model_pinocchio.cpp ../../suite.cpp ../test_robot_model.cpp)
target_link_libraries(test_robot_model_pinocchio
                      wbc-robot_models-pinocchio
                      ${base-types_LIBRARIES}
                      ${pinocchio_LIBRARIES}
                      ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})
