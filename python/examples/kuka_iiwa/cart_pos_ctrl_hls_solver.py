from wbc.core import *
from wbc.robot_models import RobotModelKDL
from wbc.scenes import VelocityScene
from wbc.solvers import HierarchicalLSSolver
from wbc.controllers import CartesianPosPDController
import time
import numpy as np

# Configure robot model
robot_model=RobotModelKDL()
r=RobotModelConfig()
r.file="../../../models/urdf/kuka/kuka_iiwa.urdf"
r.actuated_joint_names = ["kuka_lbr_l_joint_1", "kuka_lbr_l_joint_2", "kuka_lbr_l_joint_3", "kuka_lbr_l_joint_4", "kuka_lbr_l_joint_5", "kuka_lbr_l_joint_6", "kuka_lbr_l_joint_7"]
r.joint_names = r.actuated_joint_names
if robot_model.configure(r) == False:
    print("Failed to configure robot model  ")
    exit(0)

# Create solver
solver = HierarchicalLSSolver()
solver.setMaxSolverOutputNorm(10)

# Set up Tasks: Only a single, Cartesian positioning task
cfg = ConstraintConfig()
cfg.name = "tcp_pose"
cfg.root = "kuka_lbr_l_link_0"
cfg.tip = "kuka_lbr_l_tcp"
cfg.ref_frame = "kuka_lbr_l_link_0"
cfg.priority = 0
cfg.activation = 1
cfg.type = ConstraintType.cart
cfg.weights = [1]*6

# Configure WBC Scene
scene=VelocityScene(robot_model, solver)
if scene.configure([cfg]) == False:
    print("Failed to configure scene")
    exit(0)

# Configure Cartesian position controller
ctrl = CartesianPosPDController()
ctrl.setPGain([3]*6)

# Target Pose
setpoint = RigidBodyStateSE3()
setpoint.pose.position  = [0.0,0.0,0.8]
setpoint.pose.orientation = [0,0,0,1]

# Actual pose
feedback = RigidBodyStateSE3()
feedback.pose.position  = [0.0,0.0,0.0]
feedback.pose.orientation = [0,0,0,1]
control_output = RigidBodyStateSE3()

# Initial joint state
joint_state = Joints()
js = JointState()
js.position = 0.1
joint_state.elements = [js]*7
joint_state.names = robot_model.jointNames()

start = time.time()

# Control Loop
sample_time = 0.01
while np.linalg.norm(setpoint.pose.position-feedback.pose.position) > 1e-4:

    robot_model.update(joint_state)

    feedback = robot_model.rigidBodyState(cfg.root, cfg.tip)
    control_output = ctrl.update(setpoint,feedback)

    scene.setReference(cfg.name,control_output)
    qp = scene.update()
    solver_output = scene.solve(qp)

    # We have to update the joint state in this unintuitive way for now, since the [] operator for the Joints-type is not exposed to python yet....
    js = joint_state.elements
    for i in range(robot_model.noOfJoints()):
        js[i].position = js[i].position + solver_output.elements[i].speed * sample_time
    joint_state.elements = js

    print("Time: [" + str(time.time()-start) + " sec]")
    print("Ref. Pos:  %.4f %.4f %.4f"  % (setpoint.pose.position[0],setpoint.pose.position[1],setpoint.pose.position[2]))
    print("Act. Pos:  %.4f %.4f %.4f"  % (feedback.pose.position[0],feedback.pose.position[1],feedback.pose.position[2]))
    print("Ctrl. Out: %.4f %.4f %.4f"  % (control_output.twist.linear[0],control_output.twist.linear[1],control_output.twist.linear[2]))
    print("-------------------------------------------")
    time.sleep(sample_time)
