add_executable(test_qpswift_solver test_qpswift_solver.cpp ../../suite.cpp)
target_link_libraries(test_qpswift_solver
                      wbc-solvers-qpswift
                      ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})
