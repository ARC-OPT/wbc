from wbc.core import *
from wbc.robot_models.robot_model_rbdl import RobotModelRBDL
from wbc.scenes.acceleration_scene_tsid import AccelerationSceneTSID
from wbc.solvers.qpoases_solver import QPOASESSolver
from wbc.controllers import CartesianPosPDController
import time
import numpy as np

# Configure robot model
floating_base_state = RigidBodyStateSE3()
floating_base_state.pose.position       = [-0.03, 0.0, 0.9]
floating_base_state.pose.orientation    = [0,0,0,1]
floating_base_state.twist.linear        = floating_base_state.twist.angular        = [0,0,0]
floating_base_state.acceleration.linear = floating_base_state.acceleration.angular = [0,0,0]

robot_model=RobotModelRBDL()
r=RobotModelConfig()
r.file="../../../../models/rh5/urdf/rh5_legs.urdf"
r.floating_base = True
contacts = ActiveContacts()
contacts.names = ["FL_SupportCenter", "FR_SupportCenter"]
a = ActiveContact()
a.mu = 0.6
a.active = 1
contacts.elements = [a,a]
r.contact_points = contacts
if robot_model.configure(r) == False:
    print("Failed to configure robot model")
    exit(0)

# Create solver
solver = QPOASESSolver()
solver.setMaxNoWSR(100)

# Set up Tasks: Only a single, Cartesian positioning task
cfg = TaskConfig()
cfg.name = "zero_com_acceleration"
cfg.root = "world"
cfg.tip = "RH5_Root_Link"
cfg.ref_frame = "world"
cfg.priority = 0
cfg.activation = 1
cfg.type = TaskType.cart
cfg.weights = [1]*6

# Configure WBC Scene
scene=AccelerationSceneTSID(robot_model, solver, 0.001)
if scene.configure([cfg]) == False:
    print("Failed to configure scene")
    exit(0)

# Configure Cartesian position controller
ctrl = CartesianPosPDController()
ctrl.setPGain([3]*6)
ctrl.setDGain([3]*6)
ctrl.setFFGain([1]*6)

# Target Pose
setpoint = RigidBodyStateSE3()
setpoint.pose.position  = [-0.03, 0.0, 0.9]
setpoint.pose.orientation = [0,0,0,1]

# Actual pose
feedback = RigidBodyStateSE3()
feedback.pose.position  = [-0.0, 0.0, 0.0]
feedback.pose.orientation = [0,0,0,1]
control_output = RigidBodyStateSE3()

# Initial joint state
joint_state = Joints()
init_pos = [0,0,-0.2,0.4,0,-0.2,
            0,0,-0.2,0.4,0,-0.2]
# We have to update the joint state in this unintuitive way for now, since the [] operator for the Joints-type is not exposed to python yet....
elements = []
for i in init_pos:
    js = JointState()
    js.position = i
    js.speed = 0
    js.acceleration = 0
    elements.append(js)
joint_state.elements = elements
joint_state.names = robot_model.actuatedJointNames()

floating_base_state.time.microseconds=int(round(time.time()*1e6))
robot_model.update(joint_state,floating_base_state)

feedback = robot_model.rigidBodyState(cfg.root, cfg.tip)
control_output = ctrl.update(setpoint,feedback)

scene.setReference(cfg.name,control_output)
qp = scene.update()
solver_output   = scene.solve(qp)
#wrenches_output = scene.getContactWrenches()

print("----- Solver output -----")
print("Names: " + str(robot_model.actuatedJointNames()))
print("Acc:   " + str([e.acceleration for e in solver_output.elements]))
print("Tau:   " + str([e.effort for e in solver_output.elements]))
#print("Contact Wrenches")
#for name,elem in zip(wrenches_output.names, wrenches_output.elements):
#    print(name + ": Force " + str(elem.force.transpose()) + ", Torque " + str(elem.torque.transpose()))
