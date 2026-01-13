add_executable(hyper_2 hyper_2.cpp)
target_link_libraries(hyper_2
                      wbc-solvers-qpoases
                      wbc-controllers
                      wbc-scenes-acceleration_reduced_tsid
                      wbc-robot_models-pinocchio)

if(ROBOT_MODEL_HYRODYN)
    add_executable(hyper_2_hybrid hyper_2_hybrid.cpp)
    target_link_libraries(hyper_2_hybrid
                        wbc-solvers-qpoases
                        wbc-controllers
                        wbc-scenes-acceleration_reduced_tsid
                        wbc-robot_models-hyrodyn)
                        
    add_executable(hyper_2_hyrodyn hyper_2_hyrodyn.cpp)
    target_link_libraries(hyper_2_hyrodyn
                          wbc-robot_models-hyrodyn)
endif()