find_package(Boost COMPONENTS system filesystem unit_test_framework serialization REQUIRED)
add_executable(test_qpoases_solver test_qpoases_solver.cpp)
target_link_libraries(test_qpoases_solver
                      wbc-solvers-qpoases
                      Boost::unit_test_framework)
