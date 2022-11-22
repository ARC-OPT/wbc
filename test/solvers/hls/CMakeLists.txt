add_executable(test_hls_solver test_hls_solver.cpp ../../suite.cpp)
target_link_libraries(test_hls_solver
                      wbc-solvers-hls
                      Boost::unit_test_framework)
