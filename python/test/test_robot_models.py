from wbc.core import *
from wbc.robot_models import *
import numpy as np

print("Configure Robot model...")
robot_model=RobotModelHyrodyn()
r=RobotModelConfig()
r.file="../../models/urdf/rh5/rh5_one_leg.urdf"
r.submechanism_file="../../models/hyrodyn/rh5/rh5_one_leg.yml"
r.actuated_joint_names = ["LLHip1", "LLHip2", "LLHip3", "LLKnee", "LLAnkleRoll", "LLAnklePitch"]
r.joint_names = ["LLHip1", "LLHip2", "LLHip3", "LLKnee", "LLAnkleRoll", "LLAnklePitch"]
r.floating_base = False
robot_model.configure(r)
print("...done")

print("Update Robot model...")
joint_state = Joints()
joint_state.names = ["LLHip1", "LLHip2", "LLHip3", "LLKnee", "LLAnkleRoll", "LLAnklePitch"]
js = JointState()
js.position = 0
js.speed = 0.1
js.acceleration = 0
joint_state.names = ["LLHip1", "LLHip2", "LLHip3", "LLKnee", "LLAnkleRoll", "LLAnklePitch"]
joint_state.elements = [js]*len(r.actuated_joint_names)
robot_model.update(joint_state)
print("...done")

print("No of Joints is " + str(robot_model.noOfJoints()))
print("No of actuated Joints is " + str(robot_model.noOfActuatedJoints()))
print("\nJoint Names ")
joint_names = robot_model.jointNames()
for n in joint_names:
    print(n)
print("\nActuated Joint Names ")
act_joint_names = robot_model.actuatedJointNames()
for n in act_joint_names:
    print(n)

root_link = "RH5_Root_Link"
tip_link = "LLAnkle_FT"

rbs = robot_model.rigidBodyState(root_link, tip_link)
print("\nPose " + root_link + " -> " + tip_link)
print(rbs.pose.position.transpose())
print(rbs.pose.orientation.transpose())

print("\nTwist " + root_link + " -> " + tip_link)
print(rbs.twist.linear.transpose())
print(rbs.twist.angular.transpose())

print("\nAcceleration " + root_link + " -> " + tip_link)
print(rbs.acceleration.linear.transpose())
print(rbs.acceleration.angular.transpose())

print("\nSpace Jacobian " + root_link + " -> " + tip_link)
space_jacobian = robot_model.spaceJacobian("RH5_Root_Link", "LLAnkle_FT")
print(space_jacobian)

print("\nBody Jacobian " + root_link + " -> " + tip_link)
body_jacobian = robot_model.bodyJacobian("RH5_Root_Link", "LLAnkle_FT")
print(body_jacobian)

print("\nSpatial Acceleration Bias " + root_link + " -> " + tip_link)
acc_bias = robot_model.spatialAccelerationBias("RH5_Root_Link", "LLAnkle_FT")
print(acc_bias.linear.transpose())
print(acc_bias.angular.transpose())

print("\nJoint Space inertia Matrix")
inertia_mat = robot_model.jointSpaceInertiaMatrix()
print(inertia_mat)

print("\nBias Forces")
eff_bias = robot_model.biasForces()
print(eff_bias)

robot_model.setGravityVector([0,0,-9.807])
print("\nGravity Vector")
print(robot_model.getGravityVector().transpose())

sel_mat = robot_model.selectionMatrix()
print("\nSelection Matrix")
print(sel_mat)

float_base_st = robot_model.floatingBaseState()
print("\nFloating Base Pose")
print(float_base_st.pose.position.transpose())
print(float_base_st.pose.orientation.transpose())
print("\nFloating Base Twist")
print(float_base_st.twist.linear.transpose())
print(float_base_st.twist.angular.transpose())
print("\nFloating Base Acceleration")
print(float_base_st.acceleration.linear.transpose())
print(float_base_st.acceleration.angular.transpose())

robot_model.setActiveContacts(["LLAnkle_FT"])
robot_model.setContactPoints(["RH5_Root_link", "LLAnkle_FT"])
print("\nContact points")
print(robot_model.getContactPoints())
print("\nActive contacts")
print(robot_model.getActiveContacts())

print("\nRobot has link LLAnkle_FT: " + str(robot_model.hasLink("LLAnkle_FT")))
print("Robot has joint LLAnkle_FT: " + str(robot_model.hasJoint("LLAnkle_FT")))

robot_model.hasJoint("LLAnkle_FT")
