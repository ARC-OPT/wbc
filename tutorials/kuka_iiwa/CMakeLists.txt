add_executable(cart_pos_ctrl cart_pos_ctrl.cpp)
target_link_libraries(cart_pos_ctrl
                      wbc-solvers-qpoases
                      wbc-scenes-velocity_qp
                      wbc-robot_models-pinocchio
                      wbc-controllers)

add_executable(cart_pos_ctrl_weights cart_pos_ctrl_weights.cpp)
target_link_libraries(cart_pos_ctrl_weights
                      wbc-solvers-qpoases
                      wbc-scenes-velocity_qp
                      wbc-robot_models-pinocchio
                      wbc-controllers)

add_executable(cart_pos_ctrl_dynamic cart_pos_ctrl_dynamic.cpp)
target_link_libraries(cart_pos_ctrl_dynamic
                      wbc-solvers-qpoases
                      wbc-scenes-acceleration_tsid
                      wbc-robot_models-pinocchio
                      wbc-controllers)
