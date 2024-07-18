#ifndef WBC_JOINT_LIMITS_HPP
#define WBC_JOINT_LIMITS_HPP

#include <Eigen/Core>

namespace wbc { namespace types {

class JointLimits{
public:
    class Limits{
    public:
        void clear(){
            position.resize(0);
            velocity.resize(0);
            acceleration.resize(0);
            effort.resize(0);
        }
        void resize(uint n){
            position.resize(n);
            velocity.resize(n);
            acceleration.resize(n);
            effort.resize(n);
        }
        Eigen::VectorXd position;
        Eigen::VectorXd velocity;
        Eigen::VectorXd acceleration;
        Eigen::VectorXd effort;

    };

    void clear(){
        min.clear();
        max.clear();
    }

    void resize(uint n){
        min.resize(n);
        max.resize(n);
    }

    Limits min;
    Limits max;
};

}
}

#endif
