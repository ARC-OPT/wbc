add_executable(benchmark_robot_models benchmark_robot_models.cpp ../benchmarks_common.cpp)
target_link_libraries(benchmark_robot_models
                      wbc-robot_models-kdl
                      wbc-robot_models-hyrodyn
                      wbc-robot_models-pinocchio
                      wbc-robot_models-rbdl)

