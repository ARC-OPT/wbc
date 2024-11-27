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
    affiliation: "1,2"
affiliations:
 - name: German Research Center for Artificial Intelligence (DFKI), Bremen, Germany
   index: 1
 - name: University of Bremen, Bremen, Germany
   index: 2
date: 06 March 2024
bibliography: paper.bib

---

# Summary

ARC-OPT (Adaptive Robot Control using Optimization) is a C++ library for Whole-Body Control (WBC) [@Sentis2006] of complex robotic systems, such as humanoids, quadrupedal robots, or mobile manipulators.  

WBC aims to describe a robot control problem in terms of costs and constraints of a quadratic program (QP) and to design a set of feedback controllers around it, each dedicated to a specific robot tasks. In every control cycle, the QP is solved and the solution is applied to the robot's actuators. WBC is a reactive control approach, which targets redundant robots and is able to control multiple tasks simultaneously, like, e.g., grasping and balancing on a humanoid robot.

# Statement of need

ARC-OPT supports the software developer in designing robot controllers by providing configuration options for different pre-defined WBC problems. WBC is a well understood control method today, and multiple mature frameworks exits. The TSID framework [@delprete2016] implements a control algorithm for legged robots on acceleration level. Similarly, the algorithm in [@Posa2016] operates on torque level. In [@Smits2009] a generalized velocity-IK framework is implemented, which is, however, tightly coupled to the Orocos project. The IHCM Whole-Body Controller has been developed for the ATLAS robot [@Feng2015], providing control algorithms for walking and manipulation based on QPs. Drake [@drake2021] is a collection of libraries for model-based design and control of complex robots. It provides interfaces to several open-source and commercial solvers, including linear least-squares, quadratic programming, and non-linear programming. Finally, ControlIt! [@controlit2021] is a middleware built around the whole-body operational space control algorithm first introduced by Sentis and Khatib [@Sentis2006]. In contrast to the existing libraries, ARC-OPT provides unified interfaces for different WBC problems on velocity, acceleration and torque level, as well as options to benchmark different QP solvers and rigid body dynamics libraries on these problems. Finally, it provides a WBC approach for robots with parallel kinematic loops, as described in [@Mronga2022].

# Description

![ARC-OPT library overview\label{fig:overview}](wbc_overview.png)

Figure \ref{fig:overview} shows an overview of the ARC-OPT library. ARC-OPT separates the implementation of controllers, robot model, solver, and scene, which allows a modular composition of the WBC problem:

* A **controller** implements a function in the robot's task space, e.g., for maintaining balance, avoiding an obstacle, or following a trajectory. ARC-OPT provides various controllers in joint or Cartesian space, like PD-Controllers, or repulsive potential fields.
* The **scene** sets up the QP, where the costs can be configured at runtime, and the constraints are specific for the implemented scene. Different scenes are currently implemented on velocity, acceleration and torque level.
* The **robot model** computes the kinematic and dynamic information that the scene requires to set up the QP, like Jacobians, mass-inertia matrices, and gravity terms. ARC-OPT implements various robot models based on Pinocchio [@carpentier2019pinocchio], RBDL [@Felis2016], KDL [@kdl2021], and Hyrodyn [@2019_Kumar_HyRoDynApproach_IDETC].
* The **solver** solves the QP set up in the scene and generates the required control input for the robot joints. ARC-OPT provides various QP solvers based on open-source implementations, e.g.,  qpOASES [@Ferreau2014], eiquadprog [@Eiquadprog2021], proxQP [@bambade2022], and qpSwift [@pandala2019qpswift].

Apart from this, ARC-OPT implements various concepts typically used in WBC, like floating base dynamics, friction cone constraints, cost weighting, and task hierarchies.

# Example

This example shows how to set up an acceleration/torque-level WBC. Here, the tasks are formulated in the cost function. Equations of motion, rigid contacts and joint torque limits are implemented as constraints. The decision variables are the joint accelerations $\ddot{\mathbf{q}}$, joint torques $\boldsymbol{\tau}$ and contact wrenches $\mathbf{f}$. Mathematically, this can be expressed by the following QP:

$$\begin{array}{cc}
\underset{\ddot{\mathbf{q}}, \boldsymbol{\tau}, \mathbf{f}}{\text{min}} & \|\mathbf{J}\ddot{\mathbf{q}} + \dot{\mathbf{J}}\dot{\mathbf{q}} - \dot{\mathbf{v}}_d\|_2 \\
\text{s.t.} & \mathbf{H}\ddot{\mathbf{q}} + \mathbf{h} = \boldsymbol{\tau} + \mathbf{J}_c\mathbf{f}  \\
      & \mathbf{J}_{c}\ddot{\mathbf{q}} = -\dot{\mathbf{J}}_c\dot{\mathbf{q}} \\
       & \boldsymbol{\tau}_m \leq \boldsymbol{\tau} \leq \boldsymbol{\tau}_M \\
\end{array}$$

where $\mathbf{J}$ is the robot Jacobian, $\dot{\mathbf{v}}_d$ the desired task space acceleration,  $\mathbf{q},\dot{\mathbf{q}},\ddot{\mathbf{q}}$ the joint positions, velocities, and accelerations $\mathbf{H}$ the mass-inertia matrix, $\mathbf{h}$ the vector of gravity and Coriolis forces, $\boldsymbol{\tau}$ the robot joint torques, $\mathbf{f}$ the contact wrenches,  $\mathbf{J}_c$ the contact Jacobian,  and $\boldsymbol{\tau}_m,\boldsymbol{\tau}_M$ the joint torque limits. To implement a simple Cartesian position controller for, e.g., controlling the end effector of a robot manipulator, the following PD-controller can be used to generate $\dot{\mathbf{v}}_d$:

$$
\dot{\mathbf{v}}_d = \dot{\mathbf{v}}_r + \mathbf{K}_d(\mathbf{v}_r-\mathbf{v}) + \mathbf{K}_p(\mathbf{x}_r-\mathbf{x})
$$

where $\mathbf{K}_p,\mathbf{K}_d$ are gain matrices, $\mathbf{x},\mathbf{v}$ the end effector position and velocity, $\dot{\mathbf{v}}_r,\mathbf{v}_r,\mathbf{x}_r$ the reference acceleration, velocity, and position. This example is implemented for a 7 DoF KUKA iiwa robot arm in the ARC-OPT tutorials\footnote{\url{https://github.com/ARC-OPT/wbc/blob/master/tutorials/kuka_iiwa/cart_pos_ctrl_dynamic.cpp}}.

The ARC-OPT library for Whole-Body Control has been used in various scientific works [@Mronga2022],[@Mronga2021],[@Mronga2020],[@Popescu2022], and evaluated on different robots, like, e.g., the RH5 humanoid [@Eer2021DesignAA] shown in Figure \ref{fig:rh5}.

![RH5 Humanoid robot standing on one leg using the ARC-OPT library\label{fig:rh5}](rh5_one_leg.png)


# Acknowledgements

ARC-OPT is supported through grants from the German Federal Ministry of Education and Research (BMBF), grant numbers 01IW21002 (M-Rock project) and  01IW20004 (VeryHuman project).

# References
