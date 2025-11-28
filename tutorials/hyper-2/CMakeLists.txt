add_executable(hyper_2_simple hyper_2_simple.cpp)
target_link_libraries(hyper_2_simple
                      wbc-solvers-qpoases
                      wbc-controllers
                      wbc-scenes-acceleration_reduced_tsid
                      wbc-scenes-acceleration_tsid
                      wbc-robot_models-pinocchio)