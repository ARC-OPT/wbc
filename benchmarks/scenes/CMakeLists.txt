add_executable(benchmark_scenes benchmark_scenes.cpp ../benchmarks_common.cpp ../robot_models_common.cpp)
target_link_libraries(benchmark_scenes
                      wbc-solvers-hls
                      wbc-solvers-qpoases
                      wbc-scenes
                      wbc-robot_models-kdl
                      wbc-robot_models-hyrodyn
                      wbc-controllers
                      Boost::system
                      Boost::filesystem)

