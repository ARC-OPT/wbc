add_executable(cart_pos_ctrl_hls cart_pos_ctrl_hls.cpp)
target_link_libraries(cart_pos_ctrl_hls
                      wbc-solvers-hls
                      wbc-scenes
                      wbc-robot_models-rbdl
                      wbc-controllers)

add_executable(cart_pos_ctrl_qpoases cart_pos_ctrl_qpoases.cpp)
target_link_libraries(cart_pos_ctrl_qpoases
                      wbc-solvers-qpoases
                      wbc-scenes
                      wbc-robot_models-rbdl
                      wbc-controllers)

if(USE_EIQUADPROG)
add_executable(cart_pos_ctrl_eiquadprog cart_pos_ctrl_eiquadprog.cpp)
target_link_libraries(cart_pos_ctrl_eiquadprog
                      wbc-solvers-eiquadprog
                      wbc-scenes
                      wbc-robot_models-rbdl
                      wbc-controllers)
endif()

add_executable(cart_pos_ctrl_hls_weights cart_pos_ctrl_hls_weights.cpp)
target_link_libraries(cart_pos_ctrl_hls_weights
                      wbc-solvers-hls
                      wbc-scenes
                      wbc-robot_models-rbdl
                      wbc-controllers)

add_executable(cart_pos_ctrl_hls_hierarchies cart_pos_ctrl_hls_hierarchies.cpp)
target_link_libraries(cart_pos_ctrl_hls_hierarchies
                      wbc-solvers-hls
                      wbc-scenes
                      wbc-robot_models-rbdl
                      wbc-controllers)
