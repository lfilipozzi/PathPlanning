#!/usr/bin/env python3

import unittest
import os, sys
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

if __name__ == '__main__':
	unittest.main()
