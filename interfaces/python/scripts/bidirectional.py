#!/usr/bin/env python3

import os, sys
import matplotlib.pyplot as plt
import numpy as np
import math
sys.path.append(os.path.join(os.path.dirname(__file__), '..','..','..','build','lib'))
import pyplanning as nav

nav.initialize()

state_space = nav.StateSpaceSE2(nav.Pose2d(-10,-10,-math.pi), nav.Pose2d(10,10,math.pi))
map = nav.ObstacleListOccupancyMap(0.1)
state_validator = nav.StateValidatorOccupancyMap(state_space, map)

obstacle_positions = []

shape = nav.RectangleShape(10.0, 1.0)
obstacle = nav.Obstacle()
obstacle.set_shape(shape)
obstacle.set_pose(nav.Pose2d(2.0, 0.0, -math.pi / 4.0))
map.add_obstacle(obstacle)
obstacle_positions.append(obstacle.get_boundary_world_position())

shape = nav.RectangleShape(10.0, 1.0)
obstacle = nav.Obstacle()
obstacle.set_shape(shape)
obstacle.set_pose(nav.Pose2d(0.0, 7.5, -math.pi / 4.0))
map.add_obstacle(obstacle)
obstacle_positions.append(obstacle.get_boundary_world_position())

shape = nav.RectangleShape(10.0, 1.0)
obstacle = nav.Obstacle()
obstacle.set_shape(shape)
obstacle.set_pose(nav.Pose2d(-8.0, 5.0, math.pi / 2.0))
map.add_obstacle(obstacle)
obstacle_positions.append(obstacle.get_boundary_world_position())

shape = nav.RectangleShape(14.0, 1.0)
obstacle = nav.Obstacle()
obstacle.set_shape(shape)
obstacle.set_pose(nav.Pose2d(5.0, -5.0, 0.0))
map.add_obstacle(obstacle)
obstacle_positions.append(obstacle.get_boundary_world_position())

gvd = nav.GVD(map)
gvd.update()
gvd.visualize("test.ppm")

algo = nav.HybridAStar()

algo.set_init_state(nav.Pose2d(0.0, -9.0, 0.0))
algo.set_goal_state(nav.Pose2d(8.0, 8.0, 0.0))

#algo.set_init_state(nav.Pose2d(8.0, 8.0, 0.0))
#algo.set_goal_state(nav.Pose2d(0.0, -9.0, 0.0))

algo.initialize(state_validator)

algo.path_interpolation = 0.8
#algo.smoother_parameters.learning_rate = 0.01
#algo.smoother_parameters.path_weight = 0
#algo.smoother_parameters.smooth_weight = 0
#algo.smoother_parameters.voronoi_weight = 0
#algo.smoother_parameters.collision_weight = 0
#algo.smoother_parameters.curvature_weight = 0

algo.search_path()

path = algo.get_path()
(forward_graph_search_explored_path_set, reverse_graph_search_explored_path_set) = algo.get_graph_search_explored_path_set();

fig, ax = plt.subplots()
ratios = np.linspace(0.0, 1.0, 10)
for path in forward_graph_search_explored_path_set:
	poses = [path.interpolate(ratio) for ratio in ratios]
	x = [p.x() for p in poses]
	y = [p.y() for p in poses]
	_ = ax.plot(x, y, color='gray', linewidth = 0.5, label='_')

ratios = np.linspace(0.0, 1.0, 10)
for path in reverse_graph_search_explored_path_set:
	poses = [path.interpolate(ratio) for ratio in ratios]
	x = [p.x() for p in poses]
	y = [p.y() for p in poses]
	_ = ax.plot(x, y, color='red', linewidth = 0.5, label='_')

graph_search_path = algo.get_graph_search_path()
ratios = np.linspace(0.0, 1.0, 500)
poses = [graph_search_path.interpolate(ratio) for ratio in ratios]
x = [p.x() for p in poses]
y = [p.y() for p in poses]
_ = ax.plot(x, y, color='orange', linewidth = 2, label='Graph search solution')

smoothed_path = algo.get_smoothed_path()
x = [p.x() for p in smoothed_path]
y = [p.y() for p in smoothed_path]
_ = ax.plot(x, y, color='yellow', linewidth = 2, label='Smoothed path')

path = algo.get_path()
x = [p.x() for p in path]
y = [p.y() for p in path]
_ = ax.plot(x, y, color='blue', linewidth = 2, linestyle = 'dashed', label='Hybrid A* solution')

for position in obstacle_positions:
	x = [p.x() for p in position]
	y = [p.y() for p in position]
	_ = ax.fill(x, y, color='black')

_ = ax.axis('equal')
_ = ax.axis((-12,12,-12,12))
_ = ax.legend()
fig.show()



fig, ax = plt.subplots()
graph_search_path = algo.get_graph_search_path()
ratios = np.linspace(0.0, 1.0, 500)
poses = [graph_search_path.interpolate(ratio) for ratio in ratios]
x = [p.x() for p in poses]
y = [p.y() for p in poses]
_ = ax.plot(x, y, color='orange', linewidth = 2, label='Graph search solution')
fig.show()
