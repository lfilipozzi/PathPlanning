#!/usr/bin/env python3

import unittest
import os, sys

import matplotlib.pyplot as plt
import numpy as np
import math

sys.path.append(os.path.join(os.path.dirname(__file__), '..','..','..','build','lib'))
import pyplanning as nav

class TestGeometry(unittest.TestCase):

	def test_point2d(self):
		pass

	def test_pose2d(self):
		pass

	def test(self):
		state_space = nav.StateSpaceSE2(nav.Pose2d(-10,-10,-1.6), nav.Pose2d(10,10,1.6))
		map = nav.ObstacleListOccupancyMap(0.5)
		state_validator = nav.StateValidatorOccupancyMap(state_space, map)

		circle = nav.CircleShape(1.0, 10)
		obstacle = nav.Obstacle()
		obstacle.set_shape(circle)
		map.add_obstacle(obstacle)

		circle = nav.CircleShape(1.0, 10)
		obstacle = nav.Obstacle()
		obstacle.set_shape(circle)
		obstacle.set_pose(nav.Pose2d(5.0, 5.0, 0.0))
		map.add_obstacle(obstacle)

		circle = nav.CircleShape(1.0, 10)
		obstacle = nav.Obstacle()
		obstacle.set_shape(circle)
		obstacle.set_pose(nav.Pose2d(5.0, -5.0, 0.0))
		map.add_obstacle(obstacle)

		gvd = nav.GVD(map)
		gvd.update()
		gvd.visualize("test.ppm")

	def test_hybrid_a_star(self):
		nav.initialize()

		state_space = nav.StateSpaceSE2(nav.Pose2d(-10,-10,-1.6), nav.Pose2d(10,10,1.6))
		map = nav.ObstacleListOccupancyMap(0.5)#0.1
		state_validator = nav.StateValidatorOccupancyMap(state_space, map)

		shape = nav.RectangleShape(10.0, 1.0)
		obstacle = nav.Obstacle()
		obstacle.set_shape(shape)
		obstacle.set_pose(nav.Pose2d(0.0, 0.0, -math.pi / 4.0))
		map.add_obstacle(obstacle)

		shape = nav.RectangleShape(10.0, 1.0)
		obstacle = nav.Obstacle()
		obstacle.set_shape(shape)
		obstacle.set_pose(nav.Pose2d(0.0, 7.0, -math.pi / 4.0))
		map.add_obstacle(obstacle)

		shape = nav.RectangleShape(10.0, 1.0)
		obstacle = nav.Obstacle()
		obstacle.set_shape(shape)
		obstacle.set_pose(nav.Pose2d(7.0, 5.0, -math.pi / 4.0))
		map.add_obstacle(obstacle)

		shape = nav.RectangleShape(10.0, 1.0)
		obstacle = nav.Obstacle()
		obstacle.set_shape(shape)
		obstacle.set_pose(nav.Pose2d(-8.0, 5.0, math.pi / 2.0))
		map.add_obstacle(obstacle)

		shape = nav.RectangleShape(18.0, 1.0)
		obstacle = nav.Obstacle()
		obstacle.set_shape(shape)
		obstacle.set_pose(nav.Pose2d(0.0, -5.0, 0.0))
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

		x = [p.x() for p in path]
		y = [p.y() for p in path]

		plt.plot(x,y)
		plt.show()

if __name__ == '__main__':
	unittest.main()
