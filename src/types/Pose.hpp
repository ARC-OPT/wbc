#ifndef WBC_TYPES_POSE_HPP
#define WBC_TYPES_POSE_HPP

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace wbc { namespace types {

class Pose{
public:
    Eigen::Vector3d position;
    Eigen::Quaterniond orientation;
};

}
}

#endif
