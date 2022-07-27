find_package(Boost COMPONENTS system filesystem unit_test_framework REQUIRED)
include_directories(${PROJECT_SOURCE_DIR}/src)

add_executable(test_pot_field_controllers test_pot_field_controllers.cpp ../suite.cpp)
target_link_libraries(test_pot_field_controllers
                      wbc-controllers
                      ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})

add_executable(test_pos_pd_controllers test_pos_pd_controllers.cpp ../suite.cpp)
target_link_libraries(test_pos_pd_controllers
                      wbc-controllers
                      ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})


add_executable(test_pid_controllers test_pid_controllers.cpp ../suite.cpp)
target_link_libraries(test_pid_controllers
                      wbc-controllers
                      ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})
