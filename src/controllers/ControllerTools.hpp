#ifndef WBC_CONTROLLER_TOOLS_HPP
#define WBC_CONTROLLER_TOOLS_HPP

namespace base {

class Pose;
class Twist;

/** Compute the twist that translates/rotates Pose a to Pose b*/
Twist operator-(const Pose& a, const Pose& b);

}



#endif
