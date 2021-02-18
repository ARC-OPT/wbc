from wbc.core import *
from wbc.robot_models import *
import numpy as np
import nose

def run(robot_model):
    joint_names = ["LLHip1", "LLHip2", "LLHip3", "LLKnee", "LLAnkleRoll", "LLAnklePitch"]
    root_link = "RH5_Root_Link"
    tip_link = "LLAnkle_FT"
    nj = len(joint_names)
    gravity_vector = [0,0,-9.81]
    contact_points = ["RH5_Root_link","LLAnkle_FT"]
    active_contacts = ["LLAnkle_FT"]

    robot_model = RobotModelHyrodyn()

    r=RobotModelConfig()
    r.file="../../models/urdf/rh5/rh5_one_leg.urdf"
    r.submechanism_file="../../models/hyrodyn/rh5/rh5_one_leg.yml"
    r.joint_names = joint_names
    r.actuated_joint_names = joint_names
    r.floating_base = False
    assert robot_model.configure(r) == True

    joint_state = Joints()
    joint_state.names = joint_names
    js = JointState()
    js.position = js.speed = js.acceleration = 0.1
    joint_state.elements = [js]*len(joint_names)
    robot_model.update(joint_state)

    assert robot_model.noOfJoints() == nj
    assert robot_model.noOfActuatedJoints() == nj
    assert robot_model.jointNames() == joint_names
    assert robot_model.actuatedJointNames() == joint_names

    rbs = robot_model.rigidBodyState(root_link, tip_link)
    space_jacobian = robot_model.spaceJacobian(root_link, tip_link)
    body_jacobian  = robot_model.bodyJacobian(root_link, tip_link)

    twist = np.array(body_jacobian).dot(np.array([js.speed]*nj))
    assert np.all(np.isclose(twist[0:3] - rbs.twist.linear, np.array([0]*3)))
    assert np.all(np.isclose(twist[3:6] - rbs.twist.angular, np.array([0]*3)))

    bias_acc = robot_model.spatialAccelerationBias(root_link, tip_link)
    accel = np.append(bias_acc.linear,bias_acc.angular) + space_jacobian.dot(np.array([js.acceleration]*nj))
    assert np.all(np.isclose(accel[0:3] - rbs.acceleration.linear, np.array([0]*3)))
    assert np.all(np.isclose(accel[3:6] - rbs.acceleration.angular, np.array([0]*3)))

    inertia_mat = robot_model.jointSpaceInertiaMatrix()
    assert inertia_mat.shape[0] == nj
    assert inertia_mat.shape[1] == nj

    eff_bias = robot_model.biasForces()
    assert len(eff_bias) == nj

    robot_model.setGravityVector(gravity_vector)
    assert np.all(robot_model.getGravityVector().transpose() == gravity_vector)

    sel_mat = robot_model.selectionMatrix()
    assert np.all(sel_mat == np.eye(nj))

    robot_model.setContactPoints(["RH5_Root_link", "LLAnkle_FT"])
    assert robot_model.getContactPoints() == contact_points
    robot_model.setActiveContacts(active_contacts)
    assert robot_model.getActiveContacts() == active_contacts

    assert robot_model.hasLink("LLAnkle_FT") == True
    assert robot_model.hasLink("LLAnkle_F") == False
    assert robot_model.hasJoint("LLAnkle_F") == False
    assert robot_model.hasJoint("LLKnee") == True

def test_robot_model_hyrodyn():
    run(RobotModelHyrodyn())

def test_robot_model_kdl():
    run(RobotModelKDL())

if __name__ == '__main__':
    nose.run()
