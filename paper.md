---
title: 'The ARC-OPT Library for Whole-Body Control of Robotic Systems'
tags:
  - C++
  - robotics
  - control
  - Whole-Body Control
authors:
  - name: Dennis Mronga
    orcid: 0000-0002-8457-1278
    affiliation: 1
  - name: Frank Kirchner
    orcid: 0000-0002-1713-9784
    affiliation: 2
affiliations:
 - name: German Research Center for Artificial Intelligence (DFKI), Bremen, Germany
   index: 1
 - name: German Research Center for Artificial Intelligence (DFKI) and University of Bremen, Bremen, Germany
   index: 2
date: 06 March 2024
bibliography: paper.bib

---

# Summary

ARC-OPT (Adaptive Robot Control using Optimization) is a C++ library for Whole-Body Control (WBC) [@Sentis2006] of complex robotic systems, such as humanoids, quadrupedal robots, or mobile manipulators.  

WBC aims to describe a robot control problem in terms of costs and constraints of a quadratic program (QP) and design a set of feedback controllers around it, each dedicated to a specific robot tasks. In every control cycle, the QP is solved and the solution is applied to the robot's actuators. WBC is a reactive control approach, which targets redundant robots and is able to control multiple tasks simultaneously, like, e.g., grasping and balancing on a humanoid robot.

# Statement of need

ARC-OPT supports the software developer in designing robot controllers by providing configuration options for different pre-defined WBC problems. In contrast to existing libraries [@delprete2016],[@Posa2016], ARC-OPT provides unified interfaces for different WBC problems on velocity, acceleration and torque level, as well as options to benchmark different QP solvers and robot model libraries on these problems. Finally, it provides a WBC approach for robots with parallel kinematic loops, as described in [@Mronga2022].

# Description

![ARC-OPT library overview](wbc_overview.png)

ARC-OPT separates the implementation of controllers, robot models, solvers, and scenes, which allows a modular composition of the WBC problem:

* A **controller** implements a function in the robot's task space, e.g., for maintaining balance, avoiding an obstacle, or following a trajectory. ARC-OPT provides various controllers in joint or Cartesian space, like PD-Controllers, or repulsive potential fields.
* The **scene** sets up the QP, where the costs can be configured at runtime, and the constraints are specific for the implemented scene. Different scenes are currently implemented on velocity, acceleration and torque level.
* The **robot model** computes the kinematic and dynamic information that the scene requires to set up the QP, like Jacobians, mass-inertia matrices, and gravity terms. ARC-OPT implements various robot models based on Pinocchio [@carpentier2019pinocchio], RBDL [@Felis2016], KDL [@kdl2021], and Hyrodyn [@2019_Kumar_HyRoDynApproach_IDETC].
* The **solver** solves the QP set up by the scene to produce the required control input for the robot. ARC-OPT provides various QP solvers based on open-source implementations, e.g.,  qpOASES [@Ferreau2014], eiquadprog [@Eiquadprog2021], proxQP [@bambade2022], and qpSwift [@pandala2019qpswift].

# Example

This example shows how to set up an acceleration/torque-level WBC. Here, the tasks are formulated in the cost function. Equations of motion, rigid contacts and joint torque limits are implemented as constraints. The decision variables are the joint accelerations $\ddot{\mathbf{q}}$, joint torques $\boldsymbol{\tau}$ and contact wrenches $\mathbf{f}$. Mathematically, this can be expressed by the following QP:

$$\begin{array}{cc}
\underset{\ddot{\mathbf{q}}, \boldsymbol{\tau}, \mathbf{f}}{\text{min}} & \|\mathbf{J}\ddot{\mathbf{q}} + \dot{\mathbf{J}}\dot{\mathbf{q}} - \dot{\mathbf{v}}_d\|_2 \\
\text{s.t.} & \mathbf{H}\ddot{\mathbf{q}} + \mathbf{h} = \boldsymbol{\tau} + \mathbf{J}_c\mathbf{f}  \\
      & \mathbf{J}_{c}\ddot{\mathbf{q}} = -\dot{\mathbf{J}}_c\dot{\mathbf{q}} \\
       & \boldsymbol{\tau}_m \leq \boldsymbol{\tau} \leq \boldsymbol{\tau}_M \\
\end{array}$$

where $\mathbf{J}$ is the robot Jacobian, $\dot{\mathbf{v}}_d$ the desired task space acceleration,  $\mathbf{H}$ the mass-inertia matrix, $\mathbf{h}$ the vector of gravity and Coriolis terms, $\boldsymbol{\tau} the robot joint torques$, $\mathbf{f}$ the contact wrenches,  $\mathbf{J}_c$ the contact Jacobian,  $\boldsymbol{\tau}_m$ and $\boldsymbol{\tau}_M$ the joint torque limits.

To implement a Cartesian position controller, e.g., for robot manipulator, the following controller can be used to generate $\dot{\mathbf{v}}_d$:

$$
\dot{\mathbf{v}}_d = \dot{\mathbf{v}}_r + \mathbf{K}_d(\mathbf{v}_r-\mathbf{v}) + \mathbf{K}_p(\mathbf{x}_r-\mathbf{x})
$$

where $\mathbf{K}_p,\mathbf{K}_d$ are gain matrices $\mathbf{x},\mathbf{v}$ the actual position and velocity in Cartesian space, $\dot{\mathbf{v}}_r,\mathbf{v}_r,\mathbf{x}_r$ the reference acceleration, velocity, and position. This example is implemented for a 7 DoF KUKA iiwa robot arm in the ARC-OPT tutorials\footnote{\url{https://github.com/ARC-OPT/wbc/blob/master/tutorials/kuka_iiwa/cart_pos_ctrl_dynamic.cpp}}.
The ARC-OPT library for Whole-Body Control has been used in various scientific works [@Mronga2022],[@Mronga2021],[@Mronga2020],[@Popescu2022].

# Acknowledgements

ARC-OPT is supported through grants from the German Federal Ministry of Education and Research (BMBF), grant numbers 01IW21002 (M-Rock project) and  01IW20004 (VeryHuman project).

# References
