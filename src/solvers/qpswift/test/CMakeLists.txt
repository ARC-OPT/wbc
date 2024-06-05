add_executable(test_qpswift_solver test_qpswift_solver.cpp)
target_link_libraries(test_qpswift_solver
                      wbc-solvers-qpswift
                      Boost::unit_test_framework)

add_test(NAME test_qpswift_solver COMMAND test_qpswift_solver)
