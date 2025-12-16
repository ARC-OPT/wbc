add_executable(test_tools test_tools.cpp)
target_link_libraries(test_tools
                      wbc-tools
                      Boost::unit_test_framework)

add_test(NAME test_tools COMMAND test_tools)
