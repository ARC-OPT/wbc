add_executable(test_acados_solver test_acados_solver.cpp)
target_link_libraries(test_acados_solver
                      wbc-solvers-acados
                      Boost::unit_test_framework)

add_test(NAME test_acados_solver COMMAND test_acados_solver)
