add_executable(test_hpipm_solver test_hpipm_solver.cpp)
target_link_libraries(test_hpipm_solver
                      wbc-solvers-hpipm
                      Boost::unit_test_framework)

add_test(NAME test_hpipm_solver COMMAND test_hpipm_solver)
