#!/usr/bin/env python

import rospy
import math
import numpy as np
import yaml
import sys
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64
import pdb
from wall_following.msg import pid_angle_input

pub = rospy.Publisher('pid_error', pid_angle_input, queue_size=10)

# You can define constants in Python as uppercase global names like these.
MIN_DISTANCE = 0.1
MAX_DISTANCE = 30.0
MIN_ANGLE = -45.0
MAX_ANGLE = 225.0
THETA = 60

#estimated delay from command to steady state
CONTROL_DELAY_ESTIMATE = 0.5
lookDistance = 0

modeMap = {
  "center" : followCenter,
  "left" : followLeft,
  "right" : followRight
}

#historical speed, updated continuosly
lastSpeed = 0

# data: single message from topic /scan
# angle: between -45 to 225 degrees, where 0 degrees is directly to the right
# Outputs length in meters to object with angle in lidar scan field of view
def getRange(data, angle, degrees=False):
  inc = data.angle_increment * (180/3.14 if degrees else 1)
  index = int(angle / inc)
  index = np.clip([index], 0, len(data.ranges)-1)
  return data.ranges[index]

# data: single message from topic /scan
# desired_distance: desired distance to the left wall [meters]
# Outputs the PID error required to make the car follow the left wall.
def followLeft(data, desired_distance):
  b = getRange(data, 180, True)
  a = getRange(data, 180-THETA, True)
  #alpha is returned in radians
  alpha = math.atan((a*math.cos(math.radians(180-THETA)) - b)/(a*math.sin(math.radians(180-THETA))))
  d_t = b*math.cos(alpha)
  d_tplus1 = d_t + lookDistance*math.sin(alpha)
  error = desired_distance - d_tplus1
  return error

# data: single message from topic /scan
# desired_distance: desired distance to the right wall [meters]
# Outputs the PID error required to make the car follow the right wall.
def followRight(data, desired_distance):
  b = getRange(data, 0, True)
  a = getRange(data, THETA, True)
  #alpha is returned in radians
  alpha = math.atan((a*math.cos(math.radians(THETA)) - b)/(a*math.sin(math.radians(THETA))))
  d_t = b*math.cos(alpha)
  d_tplus1 = d_t + lookDistance*math.sin(alpha)
  error = desired_distance - d_tplus1
  return error

# data: single message from topic /scan
# Outputs the PID error required to make the car drive in the middle
# of the hallway.
def followCenter(data):
  # TODO: implement
  return 0.0

# Callback for receiving LIDAR data on the /scan topic.
# data: the LIDAR data, published as a list of distances to the wall.
def scan_callback(data, mode="center"):

  error = modeMap(data)

  msg = pid_angle_input()
  msg.pid_error = error
  msg.header = data.header
  pub.publish(msg)

def estimateLookDistance(data):
  lastSpeed = data
  lookDistance = lastSpeed *

# Boilerplate code to start this ROS node.
# DO NOT MODIFY!
if __name__ == '__main__':
	rospy.init_node('pid_error_node', anonymous = True)
	rospy.Subscriber("scan", LaserScan, scan_callback)
  rospy.Subscriber("/vesc/speed", Float, storeSpeed)
	rospy.spin()
