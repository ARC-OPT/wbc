pkg_search_module(base-types REQUIRED base-types)
include_directories(${base-types_INCLUDE_DIRS})
link_directories(${base-types_LIBRARY_DIRS})

add_executable(test_wbc-scenes test_wbc-scenes.cpp ../suite.cpp)
target_link_libraries(test_wbc-scenes
                      wbc-scenes
                      wbc-robot_models
                      wbc-solvers
                      ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})
