add_executable(benchmark_solvers benchmark_solvers.cpp ../benchmarks_common.cpp)
target_link_libraries(benchmark_solvers
                      wbc-solvers-qpoases
                      wbc-solvers-qpswift
                      wbc-solvers-eiquadprog
                      wbc-solvers-proxqp
                      wbc-scenes
                      wbc-robot_models-hyrodyn
                      Boost::system
                      Boost::filesystem)

