add_executable(rh5_single_leg rh5_single_leg.cpp)
target_link_libraries(rh5_single_leg
                      wbc-solvers-qpoases
                      wbc-controllers
                      wbc-scenes
                      wbc-robot_models-kdl
                      wbc-robot_models-hyrodyn)

add_executable(rh5_legs_floating_base rh5_legs_floating_base.cpp)
target_link_libraries(rh5_legs_floating_base
                      wbc-solvers-qpoases
                      wbc-scenes
                      wbc-controllers
                      wbc-robot_models-kdl
                      wbc-robot_models-hyrodyn)

add_executable(rh5_single_leg_hybrid rh5_single_leg_hybrid.cpp)
target_link_libraries(rh5_single_leg_hybrid
                      wbc-solvers-qpoases
                      wbc-scenes
                      wbc-controllers
                      wbc-robot_models-kdl
                      wbc-robot_models-hyrodyn)

