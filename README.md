[![build](https://github.com/lfilipozzi/PathPlanning/actions/workflows/build.yml/badge.svg)](https://github.com/lfilipozzi/PathPlanning/actions/workflows/build.yml)

> :warning: **This project is still under development!**

Implementation of various path-planning algorithms:
- Rapidly exloring Random Tree (RRT)
- RRT*
- Hybrid A*
- A* grid search

Hybrid A* search:
![Hybrid A*](ressources/hybrid_a_star.png)

Grid A* search:
![Hybrid A*](ressources/grid_a_star.png)

This repository provides an interface to use these algorithms in python.

Build
=========
Some dependencies are stored as submodules, to initialize them:
```shell
git submodule update --recursive --init
```
The following dependencies must be installed: 
[flann](https://github.com/flann-lib/flann), 
[Eigen](https://eigen.tuxfamily.org/index.php?title=Main_Page), 
[pybind11](https://pybind11.readthedocs.io/en/stable/index.html).

This project uses CMake:
```shell
mkdir build && cd build
cmake ..
make 
```
