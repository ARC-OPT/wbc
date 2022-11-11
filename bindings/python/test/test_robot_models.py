from wbc.core import *
from wbc.robot_models.robot_model_hyrodyn import *
from wbc.robot_models.robot_model_kdl import *
import numpy as np
import nose

def run(robot_model):
    joint_names = ["LLHip1", "LLHip2", "LLHip3", "LLKnee", "LLAnkleRoll", "LLAnklePitch"]
    root_link = "RH5_Root_Link"
    tip_link = "LLAnkle_FT"
    nj = len(joint_names)
    gravity_vector = [0,0,-9.81]
    contacts = ActiveContacts()
    contacts.names = ["RH5_Root_link", "LLAnkle_FT"]
    a = ActiveContact()
    a.mu = 0.6
    a.active = 1
    contacts.elements = [a,a]

    r=RobotModelConfig()
    r.file="../../../models/rh5/urdf/rh5_single_leg.urdf"
    r.submechanism_file="../../../models/rh5/hyrodyn/rh5_single_leg.yml"
    r.floating_base = False
    assert robot_model.configure(r) == True

    joint_state = Joints()
    joint_state.names = joint_names
    js = JointState()
    js.position = js.speed = js.acceleration = 0.1
    joint_state.elements = [js]*len(joint_names)
    robot_model.update(joint_state)

    robot_model.jointState(joint_names) == joint_state
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

    #jac_dot = robot_model.jacobianDot(root_link,tip_link)

    inertia_mat = robot_model.jointSpaceInertiaMatrix()
    assert inertia_mat.shape[0] == nj
    assert inertia_mat.shape[1] == nj

    eff_bias = robot_model.biasForces()
    assert len(eff_bias) == nj

    assert robot_model.jointNames() == joint_names
    assert robot_model.actuatedJointNames() == joint_names
    assert robot_model.independentJointNames() == joint_names
    assert robot_model.jointIndex("LLHip2") == 1
    assert robot_model.baseFrame() == "RH5_Root_Link"

    joint_limits = robot_model.jointLimits()
    joint_limits.names = joint_names

    sel_mat = robot_model.selectionMatrix()
    assert np.all(sel_mat == np.eye(nj))

    assert robot_model.hasLink("LLAnkle_FT") == True
    assert robot_model.hasLink("LLAnkle_F") == False
    assert robot_model.hasJoint("LLAnkle_F") == False
    assert robot_model.hasJoint("LLKnee") == True
    assert robot_model.hasActuatedJoint("LLKnee") == True
    assert robot_model.hasActuatedJoint("LLKnee_F") == False

    cog = robot_model.centerOfMass()

    robot_model.setActiveContacts(contacts)
    robot_model.getActiveContacts() == contacts

    assert robot_model.noOfJoints() == nj
    assert robot_model.noOfActuatedJoints() == nj

    robot_model.setGravityVector([0,0,-9.81])

    robot_model.getRobotModelConfig() == r


def test_robot_model_hyrodyn():
    run(RobotModelHyrodyn())

def test_robot_model_kdl():
    run(RobotModelKDL())

if __name__ == '__main__':
    nose.run()
