#!/usr/bin/env python3

import os, sys
import matplotlib.pyplot as plt
import numpy as np
import math
sys.path.append(os.path.join(os.path.dirname(__file__), '..','..','..','build','lib'))
import pyplanning as nav

nav.initialize()

map = nav.ObstacleListOccupancyMap(0.5)
map.initialize_size(20, 20)

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

def path_cost_fcn(origin, neighbor):
	return math.sqrt((origin.row - neighbor.row)**2 + (origin.col - neighbor.col)**2)

def heuristic_fcn(state, goal): 
	return math.sqrt((state.row - goal.row)**2 + (state.col - goal.col)**2)

init = nav.GridCellPosition(1, 1)
goal = nav.GridCellPosition(35, 35)

algo = nav.PlanarAStarGrid();
algo.set_init_state(init)
algo.set_goal_state(goal)
algo.initialize(map, path_cost_fcn, heuristic_fcn)
algo.search_path()
path = algo.get_path()
explored = algo.get_explored_states();

fig1, ax = plt.subplots()
nrows = map.rows();
ncols = map.columns()
image = np.zeros((nrows, ncols))

for cell in explored:
	image[cell.row][cell.col] = 1

for cell in path:
	image[cell.row][cell.col] = 3

_ = ax.matshow(image)
fig1.show()

algo = nav.PlanarBidirectionalAStarGrid();
algo.set_init_state(init)
algo.set_goal_state(goal)
algo.initialize(map, path_cost_fcn, heuristic_fcn)
algo.search_path()

path = algo.get_path()
(explored_forward, explored_reverse) = algo.get_explored_states();

fig2, ax = plt.subplots()
nrows = map.rows();
ncols = map.columns()
image = np.zeros((nrows, ncols))

for cell in explored_forward:
	image[cell.row][cell.col] = 1

for cell in explored_reverse:
	image[cell.row][cell.col] = 2

for cell in path:
	image[cell.row][cell.col] = 3

_ = ax.matshow(image)
fig2.show()
