---
title: Holistic MM
description: A Holistic Approach to Reactive Mobile Manipulation
---

### _Jesse Haviland, Niko Sünderhauf, and Peter Corke_


**[Preprint Avaliable Here](https://arxiv.org/abs/2010.08686)**



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

```

### Position-Based Servoing Example on an omnidirectional mobile manipulator
```python

```

* * *

### Citation

If you use this work, please cite:

```
@article{haviland2021holistic,
  author={J. {Haviland} and P. {Corke}},
  title={NEO: A Novel Expeditious Optimisation Algorithm for Reactive Motion Control of Manipulators}, 
  year={2021},
  volume={6},
  number={2},
  pages={1043-1050},
  doi={10.1109/LRA.2021.3056060}}
}
```

### Acknowledgements

This research was supported by the Queensland Universilty of Technology Centre for Robotics (QCR).

![thanks](/images/qcr.png)
