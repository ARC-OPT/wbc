#ifndef JOINT_INTEGRATOR_HPP
#define JOINT_INTEGRATOR_HPP

#include <base/commands/Joints.hpp>

enum IntegrationMethod{
    RECTANGULAR = 0,
    TRAPEZOIDAL = 1, // TODO: Implement me!
    POLYNOMIAL = 2   // TODO: Implement me!
};

namespace wbc {

/**
 * @brief The JointIntegrator class implements different numerical integrators
 */
class JointIntegrator{
    bool initialized;
    base::commands::Joints prev_cmd;
public:
    JointIntegrator() : initialized(false){}
    /**
     * @brief Performs numerical from acceleration/velocity to positions
     * @param cmd Input joint command, wil be modified by the method
     * @param cycle_time Period between two consecutive calls in seconds
     */
    void integrate(const base::samples::Joints& joint_state, base::commands::Joints &cmd, double cycle_time, IntegrationMethod method = RECTANGULAR);
    /**
     * @brief Performs numerical from acceleration/velocity to positions using rectangular method
     * @param cmd Input joint command, wil be modified by the method
     * @param cycle_time Period between two consecutive calls in seconds
     */
    void integrateRectangular(base::commands::Joints &cmd, double cycle_time);
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
