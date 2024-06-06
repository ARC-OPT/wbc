add_executable(test_eiquadprog_solver test_eiquadprog_solver.cpp)
target_link_libraries(test_eiquadprog_solver
                      wbc-solvers-eiquadprog
                      Boost::unit_test_framework)

add_test(NAME test_eiquadprog_solver COMMAND test_eiquadprog_solver)
