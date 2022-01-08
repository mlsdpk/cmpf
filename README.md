CARLA Motion Planning Framework (CMPF)
=====
[![license - apache 2.0](https://img.shields.io/:license-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)
[![Documentation Status](https://readthedocs.org/projects/cmpf/badge/?version=latest)](https://cmpf.readthedocs.io/en/latest/?badge=latest)
![Clang Format](https://github.com/mlsdpk/cmpf/actions/workflows/clang_format.yml/badge.svg)
![CI Focal](https://github.com/mlsdpk/cmpf/actions/workflows/ci_focal.yml/badge.svg)

CMPF is an open-source software framework for motion planning of autonomous vehicles in [CARLA Autonomous Driving Simulator](https://carla.org/). The framework contains various components of motion planning including route planning, behavior planning (or decision making), trajectory planning and path tracking controllers. It also provides implementations of several perception and behavior prediction plugins including state-of-the-art learning-based approaches from autonomous driving literature to be used with core motion planning modules. The framework is highly reconfigurable and users can design their own behavior trees to customize for application specific motion planning stack. For more information of the project, please visit our [documention site](https://cmpf.readthedocs.io/).

## Getting started

[__Installation__](https://cmpf.readthedocs.io/en/latest/contents/getting_started/index.html#installation) — Installation steps required to use CMPF.  