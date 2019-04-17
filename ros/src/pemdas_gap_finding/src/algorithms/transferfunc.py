#!/usr/bin/env python
import rospy
from scipy.spatial.transform import Rotation as R
import numpy as np
from geometry_msgs.msg import Vector3

def globalizePoint (distAng,trans,rot):
	r = R.from_quat(rot)
	euler = r.as_euler('zyx', degrees=True).tolist()

	dist = distAng[0]	#meters
	ang = distAng[1]	#degrees

	x = dist * np.sin(np.deg2rad(ang+euler[0])) + trans[0]
	y = dist * np.cos(np.deg2rad(ang+euler[0])) + trans[1]
	z = 0						#final z position of marker

	return (x, y, z)