# WBC - Whole-Body Control

[Code API](http://bob.dfki.uni-bremen.de/apis/dfki-control/wbc/wbc)  | [Full Documentation](https://git.hb.dfki.de/wbc/documentation/wikis/home)

WBC is C++ library for optimization-based control of redundant robots. It facilitates intuitive specification and execution of reactive robot control problems that involve multiple simultaneously running tasks. 

WBC was initiated and is currently developed at the [Robotics Innovation Center](http://robotik.dfki-bremen.de/en/startpage.html) of the [German Research Center for Artificial Intelligence (DFKI)](http://www.dfki.de) in Bremen.

<img src="doc/images/DFKI_Logo_e_schrift.jpg" alt="drawing" width="300"/>

## Motivation

WBC is a standalone library for optimization-based control of redundant robots. It contains various implementations of whole-body feedback control approaches on velocity-, acceleration- and force/torque-level. WBC is meant for controlling robots with redundant degrees of freedom, like humanoids or other legged robots with floating base, but also fixed-base systems like mobile manipulators, dual-arm systems or even simple manipulators. It is also meant for controlling multiple tasks simultaneously while taking into account the physical constraints of the robot. E.g., on a humanoid robot do ... (1) keep balance (2) Grasp an object with one arm (3) maintaining an upright body posture (4) Consider the joint torque limits,  etc... WBC is a purely reactive approach, i.e., it does not involve any motion planning or trajectory optimization. However, it can be used to stabilize trajectories coming from a motion planner or trajectory optimizer and integrate them with other objectives and physical constraints of the robot.

The library is written in C++, with Python bindings for most functionalities. The general idea of optimization-based robot control is to formulate simultaneously running robot tasks as constraints or within the cost function of an instantaneous optimization problem. 
Now, in each control cycle ...

  * The constraints/cost functions are updated with the current robot state/control reference
  * The optimization problem is solved
  * The solution is applied to the actuators of the robot

The online solution of this problem is the robot joint control signal that complies with all tasks, while integrating physical constraints like actuator limits. An advantage of this approach is that complex tasks can be composed from low-dimensional descriptors, which are typically  easier to specify and control than the complete task are once. Also, the redundancy of the robot is nicely exploited utilizing  all the dof of the system (whole body).

## Getting Started

Please check out the tutorials section in the [documentation](https://git.hb.dfki.de/dfki-control/wbc/documentation/-/wikis/home) for examples of usage.

## Requirements / Dependencies

Currently supported OS: Ubuntu18.04, Ubuntu20.04

Since WBC can be considered as an integrative framework that facilitates the composition of different robot control approaches, there is a number of dependencies:

* WBC has initially been developed for the [Robot Construction Kit (Rock)](https://www.rock-robotics.org/), thus it requires a couple of Rock core libraries, namely [base-cmake](https://github.com/rock-core/base-cmake), [base-types](https://github.com/rock-core/base-types) and [base-logging](https://github.com/rock-core/base-logging) 
* For robot modeling we use [Orocos KDL](https://github.com/orocos/orocos_kinematics_dynamics)
* For model persistence, [URDF](https://github.com/ros/urdfdom) is used, for parsing from URDF into KDL, we use the [kdl_parser](https://github.com/rock-control/control-kdl_parser)
* The only currently supported solver is [qpOASES](https://github.com/coin-or/qpOASES) 

Optional: 

* For python bindings: python-dev, libboost-python-dev, libboost-numpy-dev, python-numpy
* To support robot architectures with closed loops: [rbdl](https://git.hb.dfki.de/dfki-mechanics/hyrodyn/rbdl), [hyrodyn](https://git.hb.dfki.de/dfki-mechanics/hyrodyn/hyrodyn)

## Installation


### Standalone

Use the [install script](https://git.hb.dfki.de/dfki-control/wbc/wbc/-/tree/master/scripts):

```
wget https://git.hb.dfki.de/dfki-control/wbc/wbc/-/tree/master/scripts
sh install.sh
```

### Inside Rock

* New Bootstrap: See [here](https://git.hb.dfki.de/wbc/buildconf)
* Existing Rock Installation: Add the wbc package set to your autoproj/manifest file: 
    ```
    package_sets:
    - dfkigit: dfki-control/wbc/package_set
    ```    
  followed by `aup control/wbc` and then `amake control/wbc`

## Testing

Please check the unit tests [here](https://git.hb.dfki.de/dfki-control/wbc/wbc/-/tree/master/test), as well the comprehensive [examples](https://git.hb.dfki.de/dfki-control/wbc/wbc/-/tree/master/examples)

## Contributing

Please use the [issue tracker](https://git.hb.dfki.de/dfki-control/wbc/wbc/-/issues) to submit bug reports and feature requests.

Please use merge requests as described [here](https://git.hb.dfki.de/dfki-control/wbc/wbc/-/blob/master/CONTRIBUTING.md) to add/adapt functionality. 

## License

WBC is distributed under the [3-clause BSD license](https://opensource.org/licenses/BSD-3-Clause).

## Maintainer / Authors / Contributers

Dennis Mronga, dennis.mronga@dfki.de

Copyright 2017, DFKI GmbH / Robotics Innovation Center

