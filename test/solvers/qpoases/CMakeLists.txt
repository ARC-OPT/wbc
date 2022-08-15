add_executable(test_qpoases_solver test_qpoases_solver.cpp ../../suite.cpp)
target_link_libraries(test_qpoases_solver
                      wbc-solvers-qpoases
                      ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})
