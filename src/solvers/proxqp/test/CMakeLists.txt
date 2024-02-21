find_package(Boost COMPONENTS system filesystem unit_test_framework serialization REQUIRED)
add_executable(test_proxqp_solver test_proxqp_solver.cpp)
target_link_libraries(test_proxqp_solver
                      wbc-solvers-proxqp
                      ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})
