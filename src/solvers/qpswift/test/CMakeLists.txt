add_executable(test_qpswift_solver test_qpswift_solver.cpp)
target_link_libraries(test_qpswift_solver
                      wbc-solvers-qpswift
                      Boost::unit_test_framework)
