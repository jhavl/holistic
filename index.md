---
title: NEO
description: A Novel Expeditious Optimisation Algorithm for Reactive Motion Control of Manipulators
---

### _Jesse Haviland and Peter Corke_

IEEE Robotics and Automation Letters (RA-L), accepted January 2021.

**[Preprint Avaliable Here](https://arxiv.org/abs/2010.08686)**

**[IEEE Xplore](https://ieeexplore.ieee.org/document/9343718)**

NEO can be used on any serial-link manipulator regardless of if it is redundant or not. This includes 7 degree-of-freedom robots such as the Fanka-Emika Panda and the 6 degree-of-freedom robots such as the Universal Robotics 5 manipulator. In short, our reactive controller can avoid static and dynamic obstacles, while maximising the manipulability of the robot, avoiding joint limits and guiding the end-effector to a desired pose.

![Cover Image](/images/cover_lite.svg)

We present NEO, a fast and purely reactive motion controller for manipulators which can avoid static and dynamic obstacles while moving to the desired end-effector pose. Additionally, our controller maximises the manipulability of the robot during the trajectory, while avoiding joint position and velocity limits. NEO is wrapped into a strictly convex quadratic programme which, when considering obstacles, joint limits, and manipulability on a 7 degree-of-freedom robot, is generally solved in a few ms. While NEO is not intended to replace state-of-the-art motion planners, our experiments show that it is a viable alternative for scenes with moderate complexity while also being capable of reactive control. For more complex scenes, NEO is better suited as a  reactive local controller, in conjunction with a global motion planner. We compare NEO to motion planners on a standard benchmark in simulation and additionally illustrate and verify its operation on a physical robot in a dynamic environment. We provide an open-source library which implements our controller.

<br>

<iframe width="560" height="315" src="https://www.youtube.com/embed/jSLPJBr8QTY" frameborder="0" allow="accelerometer; autoplay; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>

<br>

We compared NEO to four state-of-the-art motion planners on a standard motion planning benchmark. As shown in the table below, NEO is a viable alternative to motion planning, while also being much faster. NEO does not perform as well as all other motion planners especially in close confined areas but does well in more open areas like bookshelves. Motion planners typically devise the whole motion before the robot starts moving making them slow and somewhat open-loop. This makes them less useful for tasks where the environment is dynamic or where the desired end-effector pose is changing. NEO, being a purely-reactive controller, excels at these scenarios. See the video above for examples of a robot being controlled by NEO dodging dynamic obstacles while tracking a moving desired end-effector pose.

|                   | NEO (ours) | Trajopt | OMPL-RRTConnect | OMPL-LBKPIECE | CHOMP-HMC |
|-------------------|------------|---------|-----------------|---------------|-----------|
| Success           | 74.2%      | 81.8%   | 85.4%           | 75.8%         | 65.2%     |
| Average Time (ms) | 9.8        | 191     | 615             | 1300          | 4910      |

The above table represents success of 198 motion problems where a 7 degree-of-freedom Personal Robot 2 (PR2) arm must get between a defined starting configuration and final pose. The 198 problems are set in five different scenes where we provide implementations of these scenes using our simulator [Swift](https://github.com/jhavl/swift) and the [Robotics Toolbox for Python](https://github.com/petercorke/robotics-toolbox-python): [Bookshelves](https://github.com/jhavl/neo/blob/master/examples/bookshelf.py), [Counter Top](https://github.com/jhavl/neo/blob/master/examples/counter.py), [Industrial A](https://github.com/jhavl/neo/blob/master/examples/industrial_a.py), [Industrial B](https://github.com/jhavl/neo/blob/master/examples/industrial_b.py), and [Tunnel](https://github.com/jhavl/neo/blob/master/examples/tunnel.py).

* * *

## How do I use it?

We have a created a robotics Python library called [Robotics Toolbox for Python](https://github.com/petercorke/robotics-toolbox-python) which allows our algorithm to be used an any robot. We use the library [qpsolvers](https://pypi.org/project/qpsolvers/) to solve the optimisation function. However, you can use whichever solver you wish.

Install [Robotics Toolbox for Python](https://github.com/petercorke/robotics-toolbox-python) from github

```shell
git clone https://github.com/petercorke/robotics-toolbox-python.git
cd robotics-toolbox-python
pip3 install -e .[swift]
```

The following example uses our Swift simulator. Install [Swift](https://github.com/jhavl/swift) from github

```shell
git clone https://github.com/jhavl/swift.git
cd swift
pip3 install -e .
```

### Position-Based Servoing Example with Two Dynamic Obstacles
```python
import roboticstoolbox as rtb
import spatialmath as sm
import numpy as np
import qpsolvers as qp

# Launch the simulator Swift
env = rtb.backends.Swift()
env.launch()

# Create a Panda robot object
panda = rtb.models.Panda()

# Set joint angles to ready configuration
panda.q = panda.qr

# Number of joint in the panda which we are controlling
n = 7

# Make two obstacles with velocities
s0 = rtb.Sphere(
    radius=0.05,
    base=sm.SE3(0.52, 0.4, 0.3)
)
s0.v = [0, -0.2, 0, 0, 0, 0]

s1 = rtb.Sphere(
    radius=0.05,
    base=sm.SE3(0.1, 0.35, 0.65)
)
s1.v = [0, -0.2, 0, 0, 0, 0]

collisions = [s0, s1]

# Make a target
target = rtb.Sphere(
    radius=0.02,
    base=sm.SE3(0.6, -0.2, 0.0)
)

# Add the Panda and shapes to the simulator
env.add(panda)
env.add(s0)
env.add(s1)
env.add(target)

# Set the desired end-effector pose to the location of target
Tep = panda.fkine(panda.q)
Tep.A[:3, 3] = target.base.t
Tep.A[2, 3] += 0.1

arrived = False

while not arrived:

    # The pose of the Panda's end-effector
    Te = panda.fkine(panda.q)

    # Transform from the end-effector to desired pose
    eTep = Te.inv() * Tep

    # Spatial error
    e = np.sum(np.abs(np.r_[eTep.t, eTep.rpy() * np.pi/180]))

    # Calulate the required end-effector spatial velocity for the robot
    # to approach the goal. Gain is set to 1.0
    v, arrived = rtb.p_servo(Te, Tep, 0.5, 0.05)

    # Gain term (lambda) for control minimisation
    Y = 0.01

    # Quadratic component of objective function
    Q = np.eye(n + 6)

    # Joint velocity component of Q
    Q[:n, :n] *= Y

    # Slack component of Q
    Q[n:, n:] = (1 / e) * np.eye(6)

    # The equality contraints
    Aeq = np.c_[panda.jacobe(panda.q), np.eye(6)]
    beq = v.reshape((6,))

    # The inequality constraints for joint limit avoidance
    Ain = np.zeros((n + 6, n + 6))
    bin = np.zeros(n + 6)

    # The minimum angle (in radians) in which the joint is allowed to approach
    # to its limit
    ps = 0.05

    # The influence angle (in radians) in which the velocity damper
    # becomes active
    pi = 0.9

    # Form the joint limit velocity damper
    Ain[:n, :n], bin[:n] = panda.joint_velocity_damper(ps, pi, n)

    # For each collision in the scene
    for collision in collisions:

        # Form the velocity damper inequality contraint for each collision
        # object on the robot to the collision in the scene
        c_Ain, c_bin = panda.link_collision_damper(
            collision, panda.q[:n], 0.3, 0.05, 1.0,
            startlink=panda.link_dict['panda_link1'],
            endlink=panda.link_dict['panda_hand'])

        # If there are any parts of the robot within the influence distance
        # to the collision in the scene
        if c_Ain is not None and c_bin is not None:
            c_Ain = np.c_[c_Ain, np.zeros((c_Ain.shape[0], 6))]

            # Stack the inequality constraints
            Ain = np.r_[Ain, c_Ain]
            bin = np.r_[bin, c_bin]

    # Linear component of objective function: the manipulability Jacobian
    c = np.r_[-panda.jacobm(panda.q).reshape((n,)), np.zeros(6)]

    # The lower and upper bounds on the joint velocity and slack variable
    lb = -np.r_[panda.qdlim[:n], 10 * np.ones(6)]
    ub = np.r_[panda.qdlim[:n], 10 * np.ones(6)]

    # Solve for the joint velocities dq
    qd = qp.solve_qp(Q, c, Ain, bin, Aeq, beq, lb=lb, ub=ub)

    # Apply the joint velocities to the Panda
    panda.qd[:n] = qd[:n]

    # Step the simulator by 50 ms
    env.step(0.05)
```

* * *

### Citation

If you use this work, please cite:

```
@article{haviland2020neo,
  author={J. {Haviland} and P. {Corke}},
  journal={IEEE Robotics and Automation Letters}, 
  title={NEO: A Novel Expeditious Optimisation Algorithm for Reactive Motion Control of Manipulators}, 
  year={2021},
  volume={6},
  number={2},
  pages={1043-1050},
  doi={10.1109/LRA.2021.3056060}}
}
```

### Acknowledgements

This research was supported by the Australian Centre for Robotic Vision (ACRV) and the Queensland Universilty of Technology Centre for Robotics (QCR).

![thanks](/images/acrvqut.png)
