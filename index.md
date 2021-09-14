---
title: Holistic MM
description: A Holistic Approach to Reactive Mobile Manipulation
---

### _Jesse Haviland, Niko Sünderhauf, and Peter Corke_


**[Preprint Avaliable Here](https://arxiv.org/abs/2109.04749)**



![Cover Image](/images/cover_mm.svg)

We present the design and implementation of a taskable reactive mobile manipulation system. Contrary to related work, we treat the arm and base degrees of freedom as a holistic structure which greatly improves the speed and fluidity of the resulting motion. At the core of this approach is a robust and reactive motion controller which can achieve a desired end-effector pose while avoiding joint position and velocity limits, and ensuring the mobile manipulator is manoeuvrable throughout the trajectory. This can support sensor-based behaviours such as closed-loop visual grasping. As no planning is involved in our approach, the robot is never stationary _thinking_ about what to do next. We show the versatility of our holistic motion controller by implementing a pick and place system using behaviour trees and demonstrate this task on a 9-degree-of-freedom mobile manipulator. Additionally, we provide an open-source implementation of our motion controller for both non-holonomic and omnidirectional mobile manipulators.

This approach can be used on both holonomic and omnidirectional mobile manipualtors. In the video below, we show it working on a 9 degree-of-freedom mobile manipualtor.

<br>

<iframe width="560" height="315" src="https://www.youtube.com/embed/-DXBQPeLIV4" frameborder="0" allow="accelerometer; autoplay; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>

<br>

* * *

## How do I use it?

We have a created a robotics Python library called [Robotics Toolbox for Python](https://github.com/petercorke/robotics-toolbox-python) which allows our algorithm to be used an any robot. The following examples uses our [Swift](https://github.com/jhavl/swift) simulator.

Install [Robotics Toolbox for Python](https://github.com/petercorke/robotics-toolbox-python) and [Swift](https://github.com/jhavl/swift) using

```shell
pip3 install roboticstoolbox-python
```

### Position-Based Servoing Example on a non-holonomic mobile manipulator
```python
import swift
import roboticstoolbox as rtb
import spatialgeometry as sg
import spatialmath as sm
import qpsolvers as qp
import numpy as np
import math


def step_robot(r, Tep):

    wTe = r.fkine(r.q, fast=True)

    eTep = np.linalg.inv(wTe) @ Tep

    # Spatial error
    et = np.sum(np.abs(eTep[:3, -1]))

    # Gain term (lambda) for control minimisation
    Y = 0.01

    # Quadratic component of objective function
    Q = np.eye(r.n + 6)

    # Joint velocity component of Q
    Q[: r.n, : r.n] *= Y
    Q[:2, :2] *= 1.0 / et

    # Slack component of Q
    Q[r.n :, r.n :] = (1.0 / et) * np.eye(6)

    v, _ = rtb.p_servo(wTe, Tep, 1.5)

    v[3:] *= 1.3

    # The equality contraints
    Aeq = np.c_[r.jacobe(r.q, fast=True), np.eye(6)]
    beq = v.reshape((6,))

    # The inequality constraints for joint limit avoidance
    Ain = np.zeros((r.n + 6, r.n + 6))
    bin = np.zeros(r.n + 6)

    # The minimum angle (in radians) in which the joint is allowed to approach
    # to its limit
    ps = 0.1

    # The influence angle (in radians) in which the velocity damper
    # becomes active
    pi = 0.9

    # Form the joint limit velocity damper
    Ain[: r.n, : r.n], bin[: r.n] = r.joint_velocity_damper(ps, pi, r.n)

    # Linear component of objective function: the manipulability Jacobian
    c = np.concatenate(
        (np.zeros(2), -r.jacobm(start=r.links[4]).reshape((r.n - 2,)), np.zeros(6))
    )

    # Get base to face end-effector
    kε = 0.5
    bTe = r.fkine(r.q, include_base=False, fast=True)
    θε = math.atan2(bTe[1, -1], bTe[0, -1])
    ε = kε * θε
    c[0] = -ε

    # The lower and upper bounds on the joint velocity and slack variable
    lb = -np.r_[r.qdlim[: r.n], 10 * np.ones(6)]
    ub = np.r_[r.qdlim[: r.n], 10 * np.ones(6)]

    # Solve for the joint velocities dq
    qd = qp.solve_qp(Q, c, Ain, bin, Aeq, beq, lb=lb, ub=ub)
    qd = qd[: r.n]

    if et > 0.5:
        qd *= 0.7 / et
    else:
        qd *= 1.4

    if et < 0.02:
        return True, qd
    else:
        return False, qd


env = swift.Swift()
env.launch(realtime=True)

ax_goal = sg.Axes(0.1)
env.add(ax_goal)

frankie = rtb.models.Frankie()
frankie.q = frankie.qr
env.add(frankie)

arrived = False
dt = 0.025

# Behind
env.set_camera_pose([-2, 3, 0.7], [-2, 0.0, 0.5])
wTep = frankie.fkine(frankie.q) * sm.SE3.Rz(np.pi)
wTep.A[:3, :3] = np.diag([-1, 1, -1])
wTep.A[0, -1] -= 4.0
wTep.A[2, -1] -= 0.25
ax_goal.base = wTep
env.step()


while not arrived:

    arrived, frankie.qd = step_robot(frankie, wTep.A)
    env.step(dt)

    # Reset bases
    base_new = frankie.fkine(frankie._q, end=frankie.links[2], fast=True)
    frankie._base.A[:] = base_new
    frankie.q[:2] = 0

env.hold()
```

### Position-Based Servoing Example on an omnidirectional mobile manipulator
```python
import swift
import roboticstoolbox as rtb
import spatialgeometry as sg
import spatialmath as sm
import qpsolvers as qp
import numpy as np
import math


def step_robot(r, Tep):

    wTe = r.fkine(r.q, fast=True)

    eTep = np.linalg.inv(wTe) @ Tep

    # Spatial error
    et = np.sum(np.abs(eTep[:3, -1]))

    # Gain term (lambda) for control minimisation
    Y = 0.01

    # Quadratic component of objective function
    Q = np.eye(r.n + 6)

    # Joint velocity component of Q
    Q[: r.n, : r.n] *= Y
    Q[:3, :3] *= 1.0 / et

    # Slack component of Q
    Q[r.n :, r.n :] = (1.0 / et) * np.eye(6)

    v, _ = rtb.p_servo(wTe, Tep, 1.5)

    v[3:] *= 1.3

    # The equality contraints
    Aeq = np.c_[r.jacobe(r.q, fast=True), np.eye(6)]
    beq = v.reshape((6,))

    # The inequality constraints for joint limit avoidance
    Ain = np.zeros((r.n + 6, r.n + 6))
    bin = np.zeros(r.n + 6)

    # The minimum angle (in radians) in which the joint is allowed to approach
    # to its limit
    ps = 0.1

    # The influence angle (in radians) in which the velocity damper
    # becomes active
    pi = 0.9

    # Form the joint limit velocity damper
    Ain[: r.n, : r.n], bin[: r.n] = r.joint_velocity_damper(ps, pi, r.n)

    # Linear component of objective function: the manipulability Jacobian
    c = np.concatenate(
        (np.zeros(3), -r.jacobm(start=r.links[5]).reshape((r.n - 3,)), np.zeros(6))
    )

    # Get base to face end-effector
    kε = 0.5
    bTe = r.fkine(r.q, include_base=False, fast=True)
    θε = math.atan2(bTe[1, -1], bTe[0, -1])
    ε = kε * θε
    c[0] = -ε

    # The lower and upper bounds on the joint velocity and slack variable
    lb = -np.r_[r.qdlim[: r.n], 10 * np.ones(6)]
    ub = np.r_[r.qdlim[: r.n], 10 * np.ones(6)]

    # Solve for the joint velocities dq
    qd = qp.solve_qp(Q, c, Ain, bin, Aeq, beq, lb=lb, ub=ub)
    qd = qd[: r.n]

    if et > 0.5:
        qd *= 0.7 / et
    else:
        qd *= 1.4

    if et < 0.02:
        return True, qd
    else:
        return False, qd


env = swift.Swift()
env.launch(realtime=True)

ax_goal = sg.Axes(0.1)
env.add(ax_goal)

frankie = rtb.models.FrankieOmni()
frankie.q = frankie.qr
env.add(frankie)

arrived = False
dt = 0.025

# Behind
env.set_camera_pose([-2, 3, 0.7], [-2, 0.0, 0.5])
wTep = frankie.fkine(frankie.q) * sm.SE3.Rz(np.pi)
wTep.A[:3, :3] = np.diag([-1, 1, -1])
wTep.A[0, -1] -= 4.0
wTep.A[2, -1] -= 0.25
ax_goal.base = wTep
env.step()


while not arrived:

    arrived, frankie.qd = step_robot(frankie, wTep.A)
    env.step(dt)

    # Reset bases
    base_new = frankie.fkine(frankie._q, end=frankie.links[3], fast=True)
    frankie._base.A[:] = base_new
    frankie.q[:3] = 0

env.hold()
```

* * *

### Citation

If you use this work, please cite:

```
@article{haviland2021holistic,
  author={J. {Haviland} and N. {Sünderhauf} and P. {Corke}},
  title={A Holistic Approach to Reactive Mobile Manipulation},
  journal={arXiv preprint arXiv:2109.04749},
  year={2021}
}
```

### Acknowledgements

This research was supported by the Queensland Universilty of Technology Centre for Robotics ([QCR](https://research.qut.edu.au/qcr/)).

![thanks](/images/qcr.png)
