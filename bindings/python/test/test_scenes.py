from wbc.robot_models.robot_model_rbdl import *
from wbc.scenes import *
from wbc.core import *
from wbc.solvers.hls_solver import *
import numpy as np
import nose

def test_velocity_scene():
    # Test General Functionality
    robot_model=RobotModelRBDL()
    r=RobotModelConfig()
    r.file="../../../models/kuka/urdf/kuka_iiwa.urdf"
    r.actuated_joint_names = ["kuka_lbr_l_joint_1", "kuka_lbr_l_joint_2", "kuka_lbr_l_joint_3", "kuka_lbr_l_joint_4", "kuka_lbr_l_joint_5", "kuka_lbr_l_joint_6", "kuka_lbr_l_joint_7"]
    r.joint_names = r.actuated_joint_names
    assert robot_model.configure(r) == True

    solver = HierarchicalLSSolver()

    joint_state = Joints()
    js = JointState()
    js.position = 1.0
    js.speed = js.acceleration = 0
    joint_state.names = r.joint_names
    joint_state.elements = [js]*len(r.actuated_joint_names)
    robot_model.update(joint_state)

    scene=VelocityScene(robot_model, solver)
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

    ref = RigidBodyStateSE3()
    ref.twist.linear  = [0.0,0.0,0.1]
    ref.twist.angular = [0.0,0.0,0.0]
    scene.setReference(cfg.name,ref)
    hqp = scene.update()
    solver_output = scene.solve(hqp)
    status = scene.updateTasksStatus()

    assert np.all(status.elements[0].y_ref[0:3] == ref.twist.linear)
    assert np.all(status.elements[0].y_ref[3:6] == ref.twist.angular)
    assert np.all(np.isclose(status.elements[0].y_solution[0:3] - ref.twist.linear,np.zeros(3)))
    assert np.all(np.isclose(status.elements[0].y_solution[3:6] - ref.twist.angular,np.zeros(3)))

    q_dot = np.array([s.speed for s in solver_output.elements])
    J = robot_model.spaceJacobian(cfg.root, cfg.tip)
    x_dot = J.dot(q_dot)
    assert np.all(np.isclose(x_dot[0:3] - ref.twist.linear,np.zeros(3)))
    assert np.all(np.isclose(x_dot[3:6] - ref.twist.angular,np.zeros(3)))

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
    q_dot = np.array([s.speed for s in solver_output.elements])
    x_dot = J.dot(q_dot)
    assert np.all(hqp.prios[0].Wy[2] == 0)
    assert np.all(x_dot[2] == 0)

if __name__ == '__main__':
    nose.run()
