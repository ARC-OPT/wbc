#ifndef WBC_TYPES_JOINT_STATE_HPP
#define WBC_TYPES_JOINT_STATE_HPP

#include <Eigen/Core>
#include <limits>

namespace wbc { namespace types {

/**
 * @brief The JointState class describes either the state or the command for a set of joints, i.e., its actual
 * or target position, velocity, acceleration and effort
 */
class JointState{
public:
    Eigen::VectorXd position;       /** m (prismatic joint) or rad (revolute joint)*/
    Eigen::VectorXd velocity;       /** m/s (prismatic joint) or rad/s (revolute joint)*/
    Eigen::VectorXd acceleration;   /** m/ss (prismatic joint) or rad/ss (revolute joint)*/

    void clear(){
        position.resize(0);
        velocity.resize(0);
        acceleration.resize(0);
    }

    /** Resize all members to size n and init elements with NaN*/
    void resize(uint n){
        position.setConstant(n,std::numeric_limits<double>::quiet_NaN());
        velocity.setConstant(n,std::numeric_limits<double>::quiet_NaN());
        acceleration.setConstant(n,std::numeric_limits<double>::quiet_NaN());
    }
};

}
}

#endif
