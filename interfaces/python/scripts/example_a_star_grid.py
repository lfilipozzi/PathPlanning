#!/usr/bin/env python3

import os, sys
import matplotlib.pyplot as plt
from matplotlib import colors
import numpy as np
import math
sys.path.append(os.path.join(os.path.dirname(__file__), '..','..','..','build','lib'))
import pyplanning as nav

nav.initialize()

map = nav.ObstacleListOccupancyMap(0.5)
map.initialize_size(20, 20)

obstacle_cells = []

shape = nav.RectangleShape(10.0, 1.0)
obstacle = nav.Obstacle()
obstacle.set_shape(shape)
obstacle.set_pose(nav.Pose2d(2.0, 0.0, -math.pi / 4.0))
map.add_obstacle(obstacle)
obstacle_cells.append(obstacle.get_boundary_grid_cell_position(map))

shape = nav.RectangleShape(10.0, 1.0)
obstacle = nav.Obstacle()
obstacle.set_shape(shape)
obstacle.set_pose(nav.Pose2d(0.0, 7.0, -math.pi / 4.0))
map.add_obstacle(obstacle)
obstacle_cells.append(obstacle.get_boundary_grid_cell_position(map))

shape = nav.RectangleShape(10.0, 1.0)
obstacle = nav.Obstacle()
obstacle.set_shape(shape)
obstacle.set_pose(nav.Pose2d(-8.0, 5.0, math.pi / 2.0))
map.add_obstacle(obstacle)
obstacle_cells.append(obstacle.get_boundary_grid_cell_position(map))

shape = nav.RectangleShape(14.0, 1.0)
obstacle = nav.Obstacle()
obstacle.set_shape(shape)
obstacle.set_pose(nav.Pose2d(5.0, -5.0, 0.0))
map.add_obstacle(obstacle)
obstacle_cells.append(obstacle.get_boundary_grid_cell_position(map))

def path_cost_fcn(origin, neighbor):
	cost = math.sqrt((origin.row - neighbor.row)**2 + (origin.col - neighbor.col)**2)
	return cost

def heuristic_fcn(state, goal): 
	cost = math.sqrt((state.row - goal.row)**2 + (state.col - goal.col)**2)
	return cost

init = nav.GridCellPosition(1, 1)
goal = nav.GridCellPosition(35, 35)

algo_uni = nav.PlanarAStarGrid();
algo_uni.set_init_state(init)
algo_uni.set_goal_state(goal)
algo_uni.initialize(map, path_cost_fcn, heuristic_fcn)
algo_uni.search_path()
path = algo_uni.get_path()
explored = algo_uni.get_explored_states();

EMPTY_COLOR = 0
EXPLORED_COLOR = 1
EXPLORED_FORWARD_COLOR = 2
EXPLORED_REVERSE_COLOR = 3
PATH_COLOR = 4
OBSTACLE_COLOR = 5

cmap = colors.ListedColormap(['white', 'dimgray', 'darkgray', 'lightgray', 'darkblue', 'black'])
cmap_bounds = [EMPTY_COLOR, EXPLORED_COLOR, EXPLORED_FORWARD_COLOR, EXPLORED_REVERSE_COLOR, PATH_COLOR, OBSTACLE_COLOR, OBSTACLE_COLOR+1]
norm = colors.BoundaryNorm(cmap_bounds, cmap.N)

nrows = map.rows();
ncols = map.columns()
image = np.zeros((ncols, nrows))

for cell in explored:
	image[cell.col][cell.row] = EXPLORED_FORWARD_COLOR

for cell in path:
	image[cell.col][cell.row] = PATH_COLOR

for cells in obstacle_cells:
	for cell in cells:
		image[cell.col][cell.row] = OBSTACLE_COLOR;

fig1, ax = plt.subplots()
_ = ax.matshow(image, cmap=cmap, norm=norm, origin='lower')
fig1.show()

#algo_bi = nav.PlanarBidirectionalAStarGrid();
#algo_bi.set_init_state(init)
#algo_bi.set_goal_state(goal)
#algo_bi.initialize(map, path_cost_fcn, heuristic_fcn)
#algo_bi.search_path()

#path = algo_bi.get_path()
#(explored_forward, explored_reverse) = algo_bi.get_explored_states();

#nrows = map.rows();
#ncols = map.columns()
#image = np.zeros((ncols, nrows))

#for cell in explored_forward:
	#image[cell.col][cell.row] = EXPLORED_FORWARD_COLOR

#for cell in explored_reverse:
	#if image[cell.col][cell.row] != EXPLORED_FORWARD_COLOR:
		#image[cell.col][cell.row] = EXPLORED_REVERSE_COLOR
	#else:
		#image[cell.col][cell.row] = EXPLORED_COLOR

#for cell in path:
	#image[cell.col][cell.row] = PATH_COLOR

#for cells in obstacle_cells:
	#for cell in cells:
		#image[cell.col][cell.row] = OBSTACLE_COLOR;

#fig2, ax = plt.subplots()
#_ = ax.matshow(image, cmap=cmap, norm=norm, origin='lower')
#fig2.show()
