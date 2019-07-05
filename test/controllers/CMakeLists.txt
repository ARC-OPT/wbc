find_package(Boost COMPONENTS system filesystem unit_test_framework REQUIRED)
include_directories(${PROJECT_SOURCE_DIR}/src)

pkg_search_module(base-types REQUIRED base-types)
include_directories(${base-types_INCLUDE_DIRS})
link_directories(${base-types_LIBRARY_DIRS})

add_executable(pot_field_controllers pot_field_controllers.cpp ../suite.cpp)
target_link_libraries(pot_field_controllers
                      wbc-controllers
                      ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})

add_executable(pos_pd_controllers pos_pd_controllers.cpp ../suite.cpp)
target_link_libraries(pos_pd_controllers
                      wbc-controllers
                      ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})


add_executable(pid_controllers pid_controllers.cpp ../suite.cpp)
target_link_libraries(pid_controllers
                      wbc-controllers
                      ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})
