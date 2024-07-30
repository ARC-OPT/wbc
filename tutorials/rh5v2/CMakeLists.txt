add_executable(rh5v2 rh5v2.cpp)
target_link_libraries(rh5v2
                      wbc-solvers-qpoases
                      wbc-scenes-acceleration_reduced_tsid
                      wbc-controllers
                      wbc-robot_models-pinocchio)

if(ROBOT_MODEL_HYRODYN)
   add_executable(rh5v2_hybrid rh5v2_hybrid.cpp)
   target_link_libraries(rh5v2_hybrid
                         wbc-solvers-qpoases
                         wbc-scenes-acceleration_reduced_tsid
                         wbc-controllers
                         wbc-robot_models-hyrodyn)
endif()                         

