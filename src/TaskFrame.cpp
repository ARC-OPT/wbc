#include "TaskFrame.hpp"

namespace wbc{

void TfToTfKDL(const TaskFrame &tf, TaskFrameKDL& tf_kdl){
    tf_kdl.jac.data = tf.jac;
    kdl_conversions::RigidBodyState2KDL(tf.pose, tf_kdl.pose);
}

void TfKDLToTf(const TaskFrameKDL& tf_kdl, TaskFrame &tf ){
    tf.jac = tf_kdl.jac.data;
    kdl_conversions::KDL2RigidBodyState(tf_kdl.pose, tf.pose);
}
}
