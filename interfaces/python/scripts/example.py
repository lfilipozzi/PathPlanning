#!/usr/bin/env python3

import os, sys
import matplotlib.pyplot as plt
import numpy as np
import math
sys.path.append(os.path.join(os.path.dirname(__file__), '..','..','..','build','lib'))
import pyplanning as nav

nav.initialize()

state_space = nav.StateSpaceSE2(nav.Pose2d(-10,-10,-math.pi), nav.Pose2d(10,10,math.pi))
map = nav.ObstacleListOccupancyMap(0.5)#0.1
state_validator = nav.StateValidatorOccupancyMap(state_space, map)

shape = nav.RectangleShape(10.0, 1.0)
obstacle = nav.Obstacle()
obstacle.set_shape(shape)
obstacle.set_pose(nav.Pose2d(2.0, 0.0, -math.pi / 4.0))
map.add_obstacle(obstacle)

shape = nav.RectangleShape(10.0, 1.0)
obstacle = nav.Obstacle()
obstacle.set_shape(shape)
obstacle.set_pose(nav.Pose2d(0.0, 7.0, -math.pi / 4.0))
map.add_obstacle(obstacle)

shape = nav.RectangleShape(10.0, 1.0)
obstacle = nav.Obstacle()
obstacle.set_shape(shape)
obstacle.set_pose(nav.Pose2d(-8.0, 5.0, math.pi / 2.0))
map.add_obstacle(obstacle)

shape = nav.RectangleShape(14.0, 1.0)
obstacle = nav.Obstacle()
obstacle.set_shape(shape)
obstacle.set_pose(nav.Pose2d(5.0, -5.0, 0.0))
map.add_obstacle(obstacle)

gvd = nav.GVD(map)
gvd.update()
gvd.visualize("test.ppm")

algo = nav.HybridAStar(state_validator)
algo.set_init_state(nav.Pose2d(0.0, -9.0, 0.0))
algo.set_goal_state(nav.Pose2d(8.0, 8.0, 0.0))
algo.initialize()
algo.search_path()

path = algo.get_path()
explored = algo.get_explored_paths();

fig, ax = plt.subplots()
ratios = np.linspace(0.0, 1.0, 10)
for path in explored:
	poses = [path.interpolate(ratio) for ratio in ratios]
	x = [p.x() for p in poses]
	y = [p.y() for p in poses]
	_ = ax.plot(x, y, color='gray', linewidth = 0.5)
	#x = [path.get_initial_state().x(), path.get_final_state().x()]
	#y = [path.get_initial_state().y(), path.get_final_state().y()]
	#_ = ax.scatter(x, y, color='gray')

path = algo.get_path()
x = [p.x() for p in path]
y = [p.y() for p in path]
_ = ax.plot(x, y, color='blue', linewidth = 2)

_ = ax.axis('equal')
fig.show()
