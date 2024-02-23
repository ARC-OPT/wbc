find_package(Boost COMPONENTS system filesystem unit_test_framework serialization REQUIRED)
add_executable(test_pot_field_controllers test_pot_field_controllers.cpp)
target_link_libraries(test_pot_field_controllers
                      wbc-controllers
                      Boost::unit_test_framework)

add_executable(test_pos_pd_controllers test_pos_pd_controllers.cpp)
target_link_libraries(test_pos_pd_controllers
                      wbc-controllers
                      Boost::unit_test_framework)


add_executable(test_pid_controllers test_pid_controllers.cpp)
target_link_libraries(test_pid_controllers
                      wbc-controllers
                      Boost::unit_test_framework)
