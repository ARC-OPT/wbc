from wbc.robot_models.robot_model_rbdl import *
from wbc.scenes.velocity_scene import *
from wbc.scenes.velocity_scene_qp import *
from wbc.scenes.acceleration_scene import *
from wbc.scenes.acceleration_scene_tsid import *
from wbc.scenes.acceleration_scene_reduced_tsid import *
from wbc.core import *
from wbc.solvers.hls_solver import *
from wbc.solvers.qpoases_solver import *
import numpy as np
import nose
#from IPython import embed

def configure_wbc(scene):
    cfg = TaskConfig()
    cfg.name = "tcp_pose"
    cfg.root = "kuka_lbr_l_link_0"
    cfg.tip = "kuka_lbr_l_tcp"
    cfg.ref_frame = "kuka_lbr_l_link_0"
    cfg.priority = 0
    cfg.activation = 1
    cfg.type = TaskType.cart
    cfg.weights = [1]*6
    constraint_config = [cfg]

    assert scene.configure(constraint_config) == True

def run_velocity_wbc(scene, robot_model):
    cfg = TaskConfig()
    cfg.name = "tcp_pose"
    cfg.root = "kuka_lbr_l_link_0"
    cfg.tip = "kuka_lbr_l_tcp"
    cfg.ref_frame = "kuka_lbr_l_link_0"
    cfg.priority = 0
    cfg.activation = 1
    cfg.type = TaskType.cart
    cfg.weights = [1]*6
    constraint_config = [cfg]

    scene.configure(constraint_config) == True

    ref = RigidBodyStateSE3()
    ref.twist.linear  = [0.0,0.0,0.1]
    ref.twist.angular = [0.0,0.0,0.0]
    scene.setReference(cfg.name,ref)
    hqp = scene.update()
    solver_output = scene.solve(hqp)
    status = scene.updateTasksStatus()

    assert np.all(status.elements[0].y_ref[0:3] == ref.twist.linear)
    assert np.all(status.elements[0].y_ref[3:6] == ref.twist.angular)
    assert np.all(np.isclose(status.elements[0].y_solution[0:3] - ref.twist.linear,np.zeros(3),atol=1e-6))
    assert np.all(np.isclose(status.elements[0].y_solution[3:6] - ref.twist.angular,np.zeros(3),atol=1e-6))

    qd = np.array([s.speed for s in solver_output.elements])
    J = robot_model.spaceJacobian(cfg.root, cfg.tip)
    xd = J.dot(qd)
    assert np.all(np.isclose(xd[0:3] - ref.twist.linear,np.zeros(3),atol=1e-6))
    assert np.all(np.isclose(xd[3:6] - ref.twist.angular,np.zeros(3),atol=1e-6))

    # Test Joint Weights
    joint_weights = JointWeights()
    joint_weights.elements = [1,1,1,0,1,1,1]
    joint_weights.names = robot_model.jointNames()

    scene.setJointWeights(joint_weights)
    assert np.all(scene.getJointWeights().elements == joint_weights.elements)
    assert np.all(scene.getJointWeights().names == joint_weights.names)
    assert np.all(scene.getActuatedJointWeights().elements == joint_weights.elements)
    assert np.all(scene.getActuatedJointWeights().names == joint_weights.names)

    hqp = scene.update()
    solver_output = scene.solve(hqp)
    assert np.all(solver_output.elements[3].speed == 0)
    assert np.all(hqp.Wq[3] == 0)

    # Test Task Weights
    scene.setTaskWeights(cfg.name,[1,1,0,1,1,1])
    hqp = scene.update()
    solver_output = scene.solve(hqp)
    qd = np.array([s.speed for s in solver_output.elements])
    xd = J.dot(qd)
    assert np.isclose(xd[2],0)

def run_acceleration_wbc(scene, robot_model):

    cfg = TaskConfig()
    cfg.name = "tcp_pose"
    cfg.root = "kuka_lbr_l_link_0"
    cfg.tip = "kuka_lbr_l_tcp"
    cfg.ref_frame = "kuka_lbr_l_link_0"
    cfg.priority = 0
    cfg.activation = 1
    cfg.type = TaskType.cart
    cfg.weights = [1]*6
    constraint_config = [cfg]

    scene.configure(constraint_config) == True

    ref = RigidBodyStateSE3()
    ref.acceleration.linear  = [0.0,0.0,0.1]
    ref.acceleration.angular = [0.0,0.0,0.0]
    scene.setReference(cfg.name,ref)
    hqp = scene.update()
    solver_output = scene.solve(hqp)
    status = scene.updateTasksStatus()


    assert np.all(status.elements[0].y_ref[0:3] == ref.acceleration.linear)
    assert np.all(status.elements[0].y_ref[3:6] == ref.acceleration.angular)
    assert np.all(np.isclose(status.elements[0].y_solution[0:3] - ref.acceleration.linear,np.zeros(3),atol=1e-6))
    assert np.all(np.isclose(status.elements[0].y_solution[3:6] - ref.acceleration.angular,np.zeros(3),atol=1e-6))

    qd  = np.array([s.speed for s in solver_output.elements])
    qdd = np.array([s.acceleration for s in solver_output.elements])
    J = robot_model.spaceJacobian(cfg.root, cfg.tip)
    acc_bias = np.array([0]*6)
    acc_bias[0:3] = robot_model.spatialAccelerationBias(cfg.root, cfg.tip).linear
    acc_bias[3:6] = robot_model.spatialAccelerationBias(cfg.root, cfg.tip).angular
    xdd = J.dot(qdd) + acc_bias
    assert np.all(np.isclose(xdd[0:3] - ref.acceleration.linear,np.zeros(3),atol=1e-6))
    assert np.all(np.isclose(xdd[3:6] - ref.acceleration.angular,np.zeros(3),atol=1e-6))

    # Test Joint Weights
    joint_weights = JointWeights()
    joint_weights.elements = [1,1,1,0,1,1,1]
    joint_weights.names = robot_model.jointNames()

    scene.setJointWeights(joint_weights)
    assert np.all(scene.getJointWeights().elements == joint_weights.elements)
    assert np.all(scene.getJointWeights().names == joint_weights.names)
    assert np.all(scene.getActuatedJointWeights().elements == joint_weights.elements)
    assert np.all(scene.getActuatedJointWeights().names == joint_weights.names)

    hqp = scene.update()
    solver_output = scene.solve(hqp)
    assert np.all(np.isclose(solver_output.elements[3].acceleration,0))
    assert np.all(hqp.Wq[3] == 0)

    # Test Task Weights
    scene.setTaskWeights(cfg.name,[1,1,0,1,1,1])
    hqp = scene.update()
    solver_output = scene.solve(hqp)

    qd  = np.array([s.speed for s in solver_output.elements])
    qdd = np.array([s.acceleration for s in solver_output.elements])
    xdd = J.dot(qdd) + acc_bias
    assert np.isclose(xdd[2],0)

def test_configure():
    robot_model=RobotModelRBDL()
    r=RobotModelConfig()
    r.file="../../../models/kuka/urdf/kuka_iiwa.urdf"
    r.actuated_joint_names = ["kuka_lbr_l_joint_1", "kuka_lbr_l_joint_2", "kuka_lbr_l_joint_3", "kuka_lbr_l_joint_4", "kuka_lbr_l_joint_5", "kuka_lbr_l_joint_6", "kuka_lbr_l_joint_7"]
    r.joint_names = r.actuated_joint_names
    assert robot_model.configure(r) == True

    qp_solver = QPOASESSolver()
    hls_solver = HierarchicalLSSolver()

    joint_state = Joints()
    js = JointState()
    js.position = 1.0
    js.speed = js.acceleration = 0
    joint_state.names = r.joint_names
    joint_state.elements = [js]*len(r.actuated_joint_names)
    robot_model.update(joint_state)

    configure_wbc(VelocityScene(robot_model, hls_solver, 0.001))
    configure_wbc(VelocitySceneQP(robot_model, qp_solver, 0.001))
    configure_wbc(AccelerationScene(robot_model, qp_solver, 0.001))
    configure_wbc(AccelerationSceneTSID(robot_model, qp_solver, 0.001))
    configure_wbc(AccelerationSceneReducedTSID(robot_model, qp_solver, 0.001))

def test_solver_output():
    robot_model=RobotModelRBDL()
    r=RobotModelConfig()
    r.file="../../../models/kuka/urdf/kuka_iiwa.urdf"
    r.actuated_joint_names = ["kuka_lbr_l_joint_1", "kuka_lbr_l_joint_2", "kuka_lbr_l_joint_3", "kuka_lbr_l_joint_4", "kuka_lbr_l_joint_5", "kuka_lbr_l_joint_6", "kuka_lbr_l_joint_7"]
    r.joint_names = r.actuated_joint_names
    assert robot_model.configure(r) == True

    qp_solver = QPOASESSolver()
    hls_solver = HierarchicalLSSolver()

    joint_state = Joints()
    js = JointState()
    js.position = 1.0
    js.speed = js.acceleration = 0.0
    joint_state.names = r.joint_names
    joint_state.elements = [js]*len(r.actuated_joint_names)
    robot_model.update(joint_state)

    run_velocity_wbc(VelocityScene(robot_model, hls_solver, 0.001), robot_model)
    run_velocity_wbc(VelocitySceneQP(robot_model, qp_solver, 0.001), robot_model)
    run_acceleration_wbc(AccelerationScene(robot_model, qp_solver, 0.001), robot_model)
    run_acceleration_wbc(AccelerationSceneTSID(robot_model, qp_solver, 0.001), robot_model)
    # Fix me: reduced TSID does not find a solution for this problem!
    #run_acceleration_wbc(AccelerationSceneReducedTSID(robot_model, qp_solver, 0.001), robot_model)


if __name__ == '__main__':
    nose.run()
