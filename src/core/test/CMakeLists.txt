add_executable(test_core test_core.cpp)
target_link_libraries(test_core
                      wbc-core
                      Boost::unit_test_framework)
