# WBC Documentation

WBC is a standalone library for optimization-based feedback control for redundant robots. It is written in C++, with Python bindings for most functionalilities. The idea of optimization-based robot control is to formulate simultaneously running robot tasks as constraints or within the cost function of an instantaneous optimization problem. 
Now, in each control cycle ...

  * The constraints/cost functions are updated with the current robot state/control reference
  * The optimization problem is solved
  * The solution is applied to the actuators of the robot

The online solution of this problem is the robot joint control signal that complies with all tasks, while integrating physical constraints like actuator limits. An advantage of this approach is that complex tasks can be composed from low-dimensional descriptors, which are typically  easier to specify and control than the complete task are once. Also, the redundancy of the robot is nicely exploited utilizing  all the dof of the system (whole body).

* [Code API](http://bob.dfki.uni-bremen.de/apis/wbc/wbc/) 
* [Full Documentation](https://git.hb.dfki.de/wbc/documentation/wikis/home)


## Installation (standalone)

See [here](https://git.hb.dfki.de/dfki-control/wbc/documentation/-/wikis/WBC%20installation%20outside%20Rock)

## Installation (using Rock)

* New Bootstrap: See [here](https://git.hb.dfki.de/wbc/buildconf)
* Existing Rock Installation: Add the wbc package set to your autoproj/manifest file: 
    ```
    package_sets:
    - dfkigit: wbc/wbc_package_set
    ```    
  followed by `aup control/wbc` and then `amake control/wbc`
 

