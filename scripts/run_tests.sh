# Controllers
echo "Testing controllers ..."
cd build/src
cd controllers/test
./test_pid_controllers
./test_pos_pd_controllers
./test_pot_field_controllers
cd ../..

# Core
echo "Testing core library ..."
cd core/test
./test_core
cd ../..

# Robot Models
echo "Testing robot models ..."
echo "Testing RobotModelPinocchio ..."
cd robot_models/pinocchio/test
./test_robot_model_pinocchio
cd ../..
if [ -d "kdl" ]; then
  echo "Testing RobotModelKDL ..."
  cd kdl/test
  ./test_robot_model_kdl
  cd ../..
fi
if [ -d "rbdl" ]; then
  echo "Testing RobotModelRBDL ..."
  cd rbdl/test
  ./test_robot_model_rbdl
  cd ../..
fi
if [ -d "hyrodyn" ]; then
  echo "Testing RobotModelHyrodyn ..."
  cd hyrodyn/test
  ./test_robot_model_hyrodyn
  cd ../..
fi
cd ..

# Scenes
echo "Testing scenes ..."

echo "Testing VelocityScene ..."
cd scenes/velocity/test
./test_velocity_scene
cd ../..

echo "Testing VelocitySceneQuadraticCost ..."
cd velocity_qp/test
./test_velocity_scene_quadratic_cost
cd ../..


echo "Testing AccelerationScene ..."
cd acceleration/test
./test_acceleration_scene
cd ../..

echo "Testing AccelerationSceneTSID ..."
cd acceleration_tsid/test
./test_acceleration_scene_tsid
cd ../..

echo "Testing AccelerationSceneReducedTSID ..."
cd acceleration_reduced_tsid/test
./test_acceleration_scene_reduced_tsid
cd ../../..

# Solvers
echo "Testing hls ..."
cd solvers/hls/test
./test_hls_solver  
cd ../..

echo "Testing QPOasesSolver ..."
cd qpoases/test
./test_qpoases_solver
cd ../..
if [ -d "eiquadprog" ]; then
  echo "Testing EiquadprogSolver ..."
  cd eiquadprog/test
  ./test_eiquadprog_solver
  cd ../..
fi
if [ -d "proxqp" ]; then
  echo "Testing ProxQPSolver ..."
  cd proxqp/test
  ./test_proxqp_solver
  cd ../..
fi
if [ -d "qpswift" ]; then
  echo "Testing QPSwiftSolver ..."
  cd qpswift/test
  ./test_qpswift_solver
  cd ../..
fi
cd ..
