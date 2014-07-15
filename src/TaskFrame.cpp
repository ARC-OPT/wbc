#include "TaskFrame.hpp"

namespace wbc{

void TfToTfKDL(const TaskFrame &tf, TaskFrameKDL& tf_kdl){
    tf_kdl.jac.data = tf.jac;
    tf_kdl.joint_names = tf.joint_names;
    tf_kdl.tf_name = tf.tf_name;
    kdl_conversions::RigidBodyState2KDL(tf.pose, tf_kdl.pose);
}

void TfKDLToTf(const TaskFrameKDL& tf_kdl, TaskFrame &tf ){
    tf.jac = tf_kdl.jac.data;
    tf.joint_names = tf_kdl.joint_names;
    tf.tf_name = tf_kdl.tf_name;
    kdl_conversions::KDL2RigidBodyState(tf_kdl.pose, tf.pose);
}
}
