add_executable(test_hls_solver test_hls_solver.cpp)
target_link_libraries(test_hls_solver
                      wbc-solvers-hls
                      Boost::unit_test_framework)

add_test(NAME test_hls_solver COMMAND test_hls_solver)
