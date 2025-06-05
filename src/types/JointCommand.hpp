#ifndef WBC_TYPES_JOINT_COMMAND_HPP
#define WBC_TYPES_JOINT_COMMAND_HPP

#include <Eigen/Core>

namespace wbc { namespace types {

class JointCommand{
public:
    Eigen::VectorXd position;       /** m (prismatic joint) or rad (revolute joint)*/
    Eigen::VectorXd velocity;       /** m/s (prismatic joint) or rad/s (revolute joint)*/
    Eigen::VectorXd acceleration;   /** m/ss (prismatic joint) or rad/ss (revolute joint)*/
    Eigen::VectorXd effort;         /** N (prismatic joint) or Nm (revolute joint)*/

    void clear(){
        position.resize(0);
        velocity.resize(0);
        acceleration.resize(0);
        effort.resize(0);
    }

    /** Resize all members to size n and init elements with NaN*/
    void resize(uint n){
        position.setConstant(n,std::numeric_limits<double>::quiet_NaN());
        velocity.setConstant(n,std::numeric_limits<double>::quiet_NaN());
        acceleration.setConstant(n,std::numeric_limits<double>::quiet_NaN());
        effort.setConstant(n,std::numeric_limits<double>::quiet_NaN());
    }
};


enum CommandMode{
    UNSET = -1,
    POSITION = 0,
    VELOCITY = 1,
    ACCELERATION = 2,
    EFFORT = 3
};


}
}

#endif // WBC_TYPES_JOINT_COMMAND_HPP
