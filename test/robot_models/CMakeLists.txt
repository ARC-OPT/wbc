find_package(Boost COMPONENTS system filesystem unit_test_framework REQUIRED)
include_directories(${PROJECT_SOURCE_DIR}/src)

pkg_search_module(base-types REQUIRED base-types)
include_directories(${base-types_INCLUDE_DIRS})
link_directories(${base-types_LIBRARY_DIRS})

add_executable(kinematic_robot_model_kdl kinematic_robot_model_kdl.cpp ../suite.cpp)
target_link_libraries(kinematic_robot_model_kdl
                      wbc-robot_models
                      ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})
