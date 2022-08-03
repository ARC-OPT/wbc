add_executable(benchmark_solvers benchmark_solvers.cpp ../benchmarks_common.cpp ../robot_models_common.cpp)
target_link_libraries(benchmark_solvers
                      wbc-solvers-hls
                      wbc-solvers-qpoases
                      wbc-solvers-qpswift
                      wbc-solvers-eiquadprog
                      wbc-scenes
                      wbc-robot_models-hyrodyn
                      wbc-robot_models-kdl
                      Boost::system
                      Boost::filesystem)

