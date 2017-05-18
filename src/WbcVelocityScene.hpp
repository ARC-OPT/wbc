#ifndef WBCVELOCITYSCENE_HPP
#define WBCVELOCITYSCENE_HPP

#include "WbcScene.hpp"

namespace wbc{

class WbcVelocityScene : public WbcScene{
protected:
    base::VectorXd solver_output;

public:
    WbcVelocityScene(){}
    virtual ~WbcVelocityScene(){}

    /** Update the wbc scene and return the current solver output*/
    virtual void setupOptProblem(RobotModel* model, OptProblem& opt_problem) = 0;
};

} // namespace wbc

#endif
