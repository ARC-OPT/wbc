#ifndef JOINT_INTEGRATOR_HPP
#define JOINT_INTEGRATOR_HPP

#include <base/commands/Joints.hpp>

enum IntegrationMethod{
    NONE = -1,
    RECTANGULAR = 0,
    TRAPEZOIDAL = 1
};

namespace wbc {

/**
 * @brief The JointIntegrator class implements different numerical integrators
 */
class JointIntegrator{
    bool initialized;
    base::commands::Joints prev_cmd;
    base::samples::Joints cur_joint_state;
public:
    JointIntegrator() : initialized(false){}
    /**
     * @brief Performs numerical from acceleration/velocity to positions
     * @param cmd Input joint command, wil be modified by the method
     * @param cycle_time Period between two consecutive calls in seconds
     * @param method Integration method to use
     * @param use_cur_state Integrate from internal (interpolated) state or from current joint state. In the first case, the integrator will not be aware of the current robot state and
     * integrated position/velocity vs. real robot position/velocity may drift apart. As a rule-of-thumb, for stiff, position-controlled robots, use_cur_state should be false, while for
     * compliant robots that have environment contacts, e.g., walking robots, use_cur_state should be true.
     */
    void integrate(const base::samples::Joints& joint_state, base::commands::Joints &cmd, double cycle_time, IntegrationMethod method = RECTANGULAR, bool use_cur_state = false);
    /**
     * @brief Performs numerical from acceleration/velocity to positions using rectangular method
     * @param cmd Input joint command, wil be modified by the method
     * @param cycle_time Period between two consecutive calls in seconds
     */
    void integrateRectangular(base::commands::Joints &cmd, double cycle_time, bool use_cur_state = false);
    /**
     * @brief Trapezoidal method for numerical integration
     * @param cmd Input joint command, wil be modified by the method
     * @param cycle_time Period between two consecutive calls in seconds
     */
    void integrateTrapezoidal(base::commands::Joints &cmd, double cycle_time, bool use_cur_state = false);
    /**
     * @brief cmdType Return the control model type POSITION/VELOCITY/ACCELERATION depending on the valid fields
     */
    base::JointState::MODE cmdMode(const base::JointState &cmd);
    /**
     * @brief Reinitialize state of the integrator
     */
    void reinit(){initialized = false;}
};

}

#endif
