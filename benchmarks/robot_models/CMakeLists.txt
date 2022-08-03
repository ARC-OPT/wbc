add_executable(benchmark_robot_models benchmark_robot_models.cpp ../benchmarks_common.cpp ../robot_models_common.cpp)
target_link_libraries(benchmark_robot_models
                      wbc-solvers-hls
                      wbc-solvers-qpoases
                      wbc-scenes
                      wbc-robot_models-kdl
                      wbc-robot_models-hyrodyn
                      wbc-controllers
                      Boost::system
                      Boost::filesystem)

