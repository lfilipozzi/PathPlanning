#!/usr/bin/env python3

import os, sys
import matplotlib.pyplot as plt
import numpy as np
import math
sys.path.append(os.path.join(os.path.dirname(__file__), '..','..','..','build','lib'))
import pyplanning as nav

nav.initialize()
steer = math.pi / 5.0
dist = 1.0

pose = nav.Pose2d(0.0, 0.0, 0.0)
model = nav.KinematicBicycleModel(2.0, 0.0)

paths = []
paths.append(nav.PathConstantSteer(model, pose,  0*steer, dist, nav.Direction.FORWARD))
paths.append(nav.PathConstantSteer(model, pose,  1*steer, dist, nav.Direction.FORWARD))
paths.append(nav.PathConstantSteer(model, pose, -1*steer, dist, nav.Direction.FORWARD))
#paths.append(nav.PathConstantSteer(model, pose,  2*steer, dist, nav.Direction.FORWARD))
#paths.append(nav.PathConstantSteer(model, pose, -2*steer, dist, nav.Direction.FORWARD))
paths.append(nav.PathConstantSteer(model, pose,  0*steer, dist, nav.Direction.BACKWARD))
paths.append(nav.PathConstantSteer(model, pose,  1*steer, dist, nav.Direction.BACKWARD))
paths.append(nav.PathConstantSteer(model, pose, -1*steer, dist, nav.Direction.BACKWARD))
#paths.append(nav.PathConstantSteer(model, pose,  2*steer, dist, nav.Direction.BACKWARD))
#paths.append(nav.PathConstantSteer(model, pose, -2*steer, dist, nav.Direction.BACKWARD))

pose = paths[1].get_final_state()
paths.append(nav.PathConstantSteer(model, pose,  0*steer, dist, nav.Direction.FORWARD))
paths.append(nav.PathConstantSteer(model, pose,  1*steer, dist, nav.Direction.FORWARD))
paths.append(nav.PathConstantSteer(model, pose, -1*steer, dist, nav.Direction.FORWARD))
#paths.append(nav.PathConstantSteer(model, pose,  2*steer, dist, nav.Direction.FORWARD))
#paths.append(nav.PathConstantSteer(model, pose, -2*steer, dist, nav.Direction.FORWARD))
paths.append(nav.PathConstantSteer(model, pose,  0*steer, dist, nav.Direction.BACKWARD))
paths.append(nav.PathConstantSteer(model, pose,  1*steer, dist, nav.Direction.BACKWARD))
paths.append(nav.PathConstantSteer(model, pose, -1*steer, dist, nav.Direction.BACKWARD))
#paths.append(nav.PathConstantSteer(model, pose,  2*steer, dist, nav.Direction.BACKWARD))
#paths.append(nav.PathConstantSteer(model, pose, -2*steer, dist, nav.Direction.BACKWARD)

fig, ax = plt.subplots()
ratios = np.linspace(0.0, 1.0, 10)
for path in paths:
	poses = [path.interpolate(ratio) for ratio in ratios]
	x = [p.x() for p in poses]
	y = [p.y() for p in poses]
	_ = ax.plot(x, y, color='gray', linewidth = 0.5)
	#x = [path.get_initial_state().x(), path.get_final_state().x()]
	#y = [path.get_initial_state().y(), path.get_final_state().y()]
	#_ = ax.scatter(x, y, color='gray')

_ = ax.axis('equal')
fig.show()
