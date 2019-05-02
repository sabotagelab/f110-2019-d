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
MIN_ANGLE = 0
MAX_ANGLE = 3*math.pi/2
THETA = math.pi/32

#estimated delay from command to steady state
CONTROL_DELAY_ESTIMATE = 0.5
lookDistance = .1
DESIRED_DISTANCE = .5

#historical speed, updated continuosly
lastSpeed = 1

# data: single message from topic /scan
# angle: between 0(far right) to 270 (far left) degrees, where 45 degrees is directly to the right
# Outputs length in meters to object with angle in lidar scan field of view
def getRange(data, angle):
  inc = data.angle_increment 
  index = int(angle / inc)
  index = np.clip(index, 0, len(data.ranges)-1)
  return data.ranges[index]

# data: single message from topic /scan
# desired_distance: desired distance to the left wall [meters]
# Outputs the PID error required to make the car follow the left wall.
def followLeft(data, desired_distance=DESIRED_DISTANCE):
  return follow(data, desired_distance, math.pi) * -1

# data: single message from topic /scan
# desired_distance: desired distance to the right wall [meters]
# Outputs the PID error required to make the car follow the right wall.
def followRight(data, desired_distance=DESIRED_DISTANCE):
  return follow(data, desired_distance, 0)
  #alpha is returned in radians

# data: single message from topic /scan
# Outputs the PID error required to make the car drive in the middle
# of the hallway.
def followCenter(data):
  d_centre = abs(followRight(data, 0)+followLeft(data, 0))/2
  b = getRange(data, math.pi/4)
  a = getRange(data, math.pi/4+THETA)
  #alpha is returned in radians
  alpha = math.atan((a*math.cos(THETA) - b)/(a*math.sin(THETA)))
  d_t = b*math.cos(alpha)
  d_t = d_centre - d_t
  d_tplus1 = d_t + lookDistance*math.sin(alpha)
  error = 0 - d_tplus1

  return error

def follow(data, desired_distance, angle):
  b = getRange(data, math.pi/4+angle)
  a = getRange(data, math.pi/4 + abs(angle-THETA))
  #alpha is returned in radians
  alpha = math.atan((a*math.cos(abs(THETA)) - b)/(a*math.sin(abs(THETA))))
  d_t = b*math.cos(alpha)
  d_tplus1 = d_t + lookDistance*math.sin(alpha)
  error = (desired_distance - d_tplus1)
  return error

# Callback for receiving LIDAR data on the /scan topic.
# data: the LIDAR data, published as a list of distances to the wall.
modeMap = {
  "center" : followCenter,
  "left" : followLeft,
  "right" : followRight
}
def scan_callback(data, mode="left"):
  error = modeMap[mode](data)

  msg = pid_angle_input()
  msg.pid_error = error
  msg.header = data.header
  pub.publish(msg)

def estimateLookDistance(data):
  lastSpeed = data
  lookDistance = lastSpeed * lastSpeed

# Boilerplate code to start this ROS node.
# DO NOT MODIFY!
if __name__ == '__main__':
	rospy.init_node('pid_error_node', anonymous = True)
	rospy.Subscriber("scan", LaserScan, scan_callback)
  #rospy.Subscriber("/vesc/speed", Float, estimateLookDistance)
	rospy.spin()
