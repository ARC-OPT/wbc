add_executable(test_daqp_solver test_daqp_solver.cpp)
target_link_libraries(test_daqp_solver
                      wbc-solvers-daqp
                      Boost::unit_test_framework)

add_test(NAME test_daqp_solver COMMAND test_daqp_solver)
