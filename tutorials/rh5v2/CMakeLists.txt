add_executable(rh5v2 rh5v2.cpp)
target_link_libraries(rh5v2
                      wbc-solvers-qpoases
                      wbc-scenes
                      wbc-controllers
                      wbc-robot_models-rbdl)

if(USE_HYRODYN)
   add_executable(rh5v2_hybrid rh5v2_hybrid.cpp)
   target_link_libraries(rh5v2_hybrid
                         wbc-solvers-qpoases
                         wbc-scenes
                         wbc-controllers
                         wbc-robot_models-hyrodyn)
endif()                         

