from wbc.controllers import *
from wbc.core import *
import numpy as np
import nose

def test_cartesian_pos_pd_controller():
    ctrl = CartesianPosPDController()
    p_gain = [1]*6
    d_gain = [0.1]*6
    ff_gain = [0.5]*6
    max_ctrl_out = [10]*6
    dead_zone = [0]*6

    ctrl.setPGain(p_gain)
    ctrl.setDGain(d_gain)
    ctrl.setFFGain(ff_gain)
    ctrl.setMaxCtrlOutput(max_ctrl_out)
    ctrl.setDeadZone(dead_zone)
    assert np.all(ctrl.pGain() == p_gain)
    assert np.all(ctrl.dGain() == d_gain)
    assert np.all(ctrl.ffGain() == ff_gain)
    assert np.all(ctrl.maxCtrlOutput() == max_ctrl_out)
    assert np.all(ctrl.deadZone() == dead_zone)

    ref = RigidBodyStateSE3()
    ref.pose.position = [1,2,3]
    ref.pose.orientation = [0,0,0,1]
    ref.twist.linear = [1,1,1]
    ref.twist.angular = [1,1,1]

    act = RigidBodyStateSE3()
    act.pose.position = [0,0,0]
    act.pose.orientation = [0,0,0,1]

    control_output = ctrl.update(ref,act)
    assert np.all(control_output.twist.linear == [1.1, 2.1, 3.1])
    assert np.all(control_output.twist.angular == [0.1, 0.1, 0.1])

def test_joint_pos_pd_controller():
    joint_names = ["joint_1","joint_2","joint_3"]
    nj = len(joint_names)
    ctrl = JointPosPDController(joint_names)
    p_gain = [1]*nj
    d_gain = [0.1]*nj
    ff_gain = [0.1]*nj
    max_ctrl_out = [10]*nj
    dead_zone = [0]*nj

    ctrl.setPGain(p_gain)
    ctrl.setDGain(d_gain)
    ctrl.setFFGain(ff_gain)
    ctrl.setMaxCtrlOutput(max_ctrl_out)
    ctrl.setDeadZone(dead_zone)
    assert np.all(ctrl.pGain() == p_gain)
    assert np.all(ctrl.dGain() == d_gain)
    assert np.all(ctrl.ffGain() == ff_gain)
    assert np.all(ctrl.maxCtrlOutput() == max_ctrl_out)
    assert np.all(ctrl.deadZone() == dead_zone)

    js = JointState()
    js.position = 1
    js.speed = 1
    ref = Joints()
    ref.names = joint_names
    ref.elements = [js]*nj

    act = Joints()
    js.position = 0
    js.speed = 0
    act.names = joint_names
    act.elements = [js]*nj

    control_output = ctrl.update(ref,act)
    vel = np.array([c.speed for c in control_output.elements])
    assert np.all(np.isclose(vel - [1.1]*nj,np.zeros(nj),rtol=1e-5,atol=1e-7))

if __name__ == '__main__':
    nose.run()
