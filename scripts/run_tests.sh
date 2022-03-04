cd build/test/controllers
./test_pid_controllers
./test_pos_pd_controllers
./test_pot_field_controllers
cd ../core
./test_core
cd ../robot_models/kdl
./test_robot_model_kdl
cd ../../scenes
./test_velocity_scene
./test_velocity_scene_quadratic_cost
./test_acceleration_scene
cd ../solvers/hls 
./test_hls_solver
cd ../qpoases
./test_qpoases_solver
