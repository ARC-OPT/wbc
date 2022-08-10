add_executable(benchmark_scenes benchmark_scenes.cpp ../benchmarks_common.cpp)
target_link_libraries(benchmark_scenes
                      wbc-solvers-qpoases
                      wbc-solvers-qpswift
                      wbc-solvers-eiquadprog
                      wbc-scenes
                      wbc-robot_models-kdl
                      wbc-robot_models-hyrodyn
                      wbc-robot_models-rbdl
                      wbc-robot_models-pinocchio
                      Boost::system
                      Boost::filesystem)

