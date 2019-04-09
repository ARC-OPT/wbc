#ifndef WBC_TYPES_CARTESIAN_STATE_HPP
#define WBC_TYPES_CARTESIAN_STATE_HPP

#include <base/Pose.hpp>
#include <base/Time.hpp>
#include "Twist.hpp"
#include "Acceleration.hpp"

namespace wbc {

class CartesianState{
public:
    CartesianState(){
        setNaN();
    }
    /** Set all members to NaN*/
    void setNaN();
    /** Check if the pose is valid, e.g. not NaN*/
    bool hasValidPose() const;
    /** Check if the twist is valid, e.g. not NaN*/
    bool hasValidTwist() const;
    /** Check if the acceleration is valid, e.g. not NaN*/
    bool hasValidAcceleration() const;

    base::Time time;
    /** Source frame of the transform described by the given pose. */
    std::string source_frame;
    /** Target frame of the transform described by the given pose. Note that we have source-in-target convetion, i.e. the transform given by pose
      * transforms a vector from source to target frame. In this regard pose also decribes the position and orientation of the source frame wrt. the target frame*/
    std::string target_frame;
    /** 3D position and orientation*/
    base::Pose pose;
    /** 3D Linear and angular velocity*/
    Twist twist;
    /** 3D Linear and angular acceleration*/
    Acceleration acceleration;
};

Twist operator-(const base::Pose& a, const base::Pose& b);
Acceleration operator-(const Twist&a, const Twist &b);
Acceleration operator+(const Twist&a, const Acceleration &b);

}

#endif
