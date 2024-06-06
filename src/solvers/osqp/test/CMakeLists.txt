add_executable(test_osqp_solver test_osqp_solver.cpp)
target_link_libraries(test_osqp_solver
                      wbc-solvers-osqp
                      Boost::unit_test_framework)

add_test(NAME test_osqp_solver COMMAND test_osqp_solver)
