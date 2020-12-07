# WBC Documentation

### [Code API](http://bob.dfki.uni-bremen.de/apis/wbc/wbc/) | [Documentation](https://git.hb.dfki.de/wbc/documentation/wikis/home)

WBC (Whole Body Control) is an approach for specifying and controlling complex robotic tasks based on *constraints*. 
The term was coined by Luis Sentis in his phd thesis 
[Synthesis and Control of Whole-Body Behaviors in Humanoid Systems](http://citeseerx.ist.psu.edu/viewdoc/download?doi=10.1.1.73.8747&rep=rep1&type=pdf). 
However, it is not limited to humanoid robot control, but can be 
used for arbitrary robot control tasks like dual arm grasping, force-position control, mobile manipulation. 

![](doc/images/wbc_principle.png)

The fundamental idea is to break down the overall control problem into multiple, simultaneously running subtasks. Each subtask is described as a *constraint* to an optimization problem.
Now, in each control cycle ...
  * The constraints are updated with the current robot state
  * The optimization problem is solved and ...
  * The solution is applied to the actuators of the robot
  
An advantage of this approach is that complex tasks can be composed from low-dimensional descriptors (aka constraints), which are typically 
easier to specify and control than the complete task are once. Also, the redundancy of the robot is exploited automatically utilizing 
all the dof of the system (whole body). 

### Installation Under Rock

* New Bootstrap: See [here](https://git.hb.dfki.de/wbc/buildconf)
* Existing Rock Installation: Add the wbc package set to your autoproj/manifest file: 
    ```
    package_sets:
    - dfkigit: wbc/wbc_package_set
    ```    
  followed by `aup control/wbc` and then `amake control/wbc`
 
### Installation Without Rock
* See [here](https://git.hb.dfki.de/wbc/documentation/-/wikis/WBC%20Install%20Instruction%20outside%20of%20ROCK)




