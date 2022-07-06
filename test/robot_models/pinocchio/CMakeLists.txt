find_package(Boost COMPONENTS system filesystem unit_test_framework REQUIRED)
include_directories(${PROJECT_SOURCE_DIR}/src)

pkg_search_module(base-types REQUIRED base-types)
pkg_search_module(pinocchio REQUIRED pinocchio)
pkg_search_module(kdl_parser REQUIRED kdl_parser)

include_directories(${base-types_INCLUDE_DIRS} ${pinocchio_INCLUDE_DIRS} ${kdl_parser_INCLUDE_DIRS})
link_directories(${base-types_LIBRARY_DIRS} ${pinocchio_LIBRARY_DIRS} ${kdl_parser_LIBRARY_DIRS})

add_executable(test_robot_model_pinocchio test_robot_model_pinocchio.cpp ../../suite.cpp)
target_link_libraries(test_robot_model_pinocchio
                      wbc-robot_models-pinocchio
                      wbc-robot_models-kdl
                      ${base-types_LIBRARIES}
                      ${kdl_parser_LIBRARIES}
                      ${pinocchio_LIBRARIES}
                      ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})
