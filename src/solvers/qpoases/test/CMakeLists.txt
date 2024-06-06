add_executable(test_qpoases_solver test_qpoases_solver.cpp)
target_link_libraries(test_qpoases_solver
                      wbc-solvers-qpoases
                      Boost::unit_test_framework)

add_test(NAME test_qpoases_solver COMMAND test_qpoases_solver)
