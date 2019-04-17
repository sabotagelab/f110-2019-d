#!/usr/bin/env python
import rospy
from scipy.spatial.transform import Rotation as R
import numpy as np
from geometry_msgs.msg import Vector3

def globalizePoint (distAng,trans,rot):
	r = R.from_quat(rot)
	euler = r.as_euler('zyx', degrees=False).tolist()

	dist = distAng[0]	#meters
	ang = distAng[1]	#radians

	x = dist * np.sin(ang+euler[0]) + trans[0]
	y = dist * np.cos(ang+euler[0]) + trans[1]

		#final x position of marker
		#final y position of marker
	z = 0						#final z position of marker
	return (x, y, z)