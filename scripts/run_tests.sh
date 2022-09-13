# Controllers
echo "Testing controllers ..."
cd build/test/controllers
#./test_pid_controllers
#./test_pos_pd_controllers
#./test_pot_field_controllers

# Core
echo "Testing core library ..."
cd ../core
./test_core

# Robot Models
echo "Testing robot models ..."
echo "Testing RobotModelKDL ..."
cd ../robot_models/kdl
./test_robot_model_kdl
cd ..
if [ -d "pinocchio" ]; then
  echo "Testing RobotModelPinocchio ..."
  cd pinocchio
  ./test_robot_model_pinocchio
fi

# Scenes
echo "Testing scenes ..."
cd ../../scenes
echo "Testing VelocityScene ..."
./test_velocity_scene
echo "Testing VelocitySceneQuadraticCost ..."
./test_velocity_scene_quadratic_cost
echo "Testing AccelerationSceneTSID ..."
./test_acceleration_scene

# Solvers
echo "Testing hls ..."
cd ../solvers/hls 
./test_hls_solver  
echo "Testing QPOasesSolver ..."
cd ../qpoases
./test_qpoases_solver
cd ..
if [ -d "eiquadprog" ]; then
  echo "Testing EiquadprogSolver ..."
  cd eiquadprog 
  ./test_eiquadprog_solver
  cd ..
fi
if [ -d "qpswift" ]; then
  echo "Testing QPSwiftSolver ..."
  cd qpswift   
  ./test_qpswift_solver
fi
