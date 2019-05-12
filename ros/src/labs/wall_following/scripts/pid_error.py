#!/usr/bin/env python
import rospy
import math
import time
import numpy as np
import sys

#msg imports
from wall_following.msg import pid_angle_input, follow_type
from race.msg import drive_param
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64

#config imports
import yaml
import os


configFile = "config.yaml"
if len(sys.argv) > 1:
    configFile = sys.argv[1]
dirname = os.path.dirname(__file__)
filepath = os.path.join(dirname, '../config/' + configFile)

with open (filepath, 'r') as f:
  doc = yaml.load(f)
  doc = doc["pid_error"]

#ANGLE CONFIG
MIN_ANGLE = 0
MAX_ANGLE = 3*math.pi/2
THETA = math.pi/16

#DISTANCE CONFIG
DESIRED_DISTANCE = doc["DESIRED_DISTANCE"]
MIN_DISTANCE = doc["MIN_DISTANCE"]
MAX_DISTANCE = doc["MAX_DISTANCE"]

#LOOK CONFIG
#estimated delay from command to steady state
CONTROL_DELAY_ESTIMATE = doc["CONTROL_DELAY_ESTIMATE"]
minLookDistance = doc["minLookDistance"] if "minLookDistance" in doc else 0
lookDistanceMultiplier = doc["lookDistanceMultiplier"] if "lookDistanceMultiplier" in doc else 1
lookDistance = 1
currentSpeed = 1 #last speed

modeMap = {
  "center" : 0,
  "left" : 1,
  "right" : 2,
  "gap" : 3
}

#MODE CONFIG
currentMode = doc["currentMode"] if "currentMode" in doc else None
currentEnumMode = doc["currentEnumMode"] if not currentMode else modeMap[currentMode]
currentGapAngle = doc["currentGapAngle"]

class Interface:
  def __init__(self):
    rospy.init_node('pid_error_node', anonymous = True)
    self.pidErrorPub = rospy.Publisher('pid_error', pid_angle_input, queue_size=10)
    self.laserScanSub = rospy.Subscriber("filter_scan", LaserScan, self.scan_callback)
    self.followTypePub = rospy.Subscriber("follow_types", follow_type, self.changeFollowType)
    self.cmdVelSub = rospy.Subscriber("cmd_vel", drive_param, self.storeSpeed)
  
    self.currentRanges = [1] * 2000

    self.modeEnumMap = dict([
      (0 , self.followCenter),
      (1 , self.followLeft),
      (2 , self.followRight),
      (3 , self.followGap)
    ])

    self.lastTime = time.time()

  def start(self):
    rospy.spin()

  # Callback for receiving LIDAR data on the /scan topic.
  # data: the LIDAR data, published as a list of distances to the wall.
  def scan_callback(self, data):
    self.currentTime = time.time()
    self.frameTime = self.currentTime - self.lastTime
    self.lastTime = self.currentTime

    self.filterScan = data

    self.setLookDistance()

    error = self.modeEnumMap[currentEnumMode]()
    msg = pid_angle_input()
    msg.pid_error = error
    msg.header = data.header
    self.pidErrorPub.publish(msg)

  def changeFollowType(self, data):
    currentEnumMode = data.type
    currentGapAngle = data.gap_angle

  def setLookDistance(self):
    lookDistance = minLookDistance + lookDistanceMultiplier * (CONTROL_DELAY_ESTIMATE + self.frameTime) * currentSpeed
  
  def storeSpeed(self, data):
    currentSpeed = data

  # data: single message from topic /scan
  # angle: between 0(far right) to 270 (far left) degrees, where 45 degrees is directly to the right
  # Outputs length in meters to object with angle in lidar scan field of view
  def getRange(self, angle):
    inc = self.filterScan.angle_increment
    index = int(angle / inc)
    index = np.clip(index, 0, len(self.filterScan.ranges)-1)
    result = self.filterScan.ranges[index]
    return result

  # data: single message from topic /scan
  # desired_distance: desired distance to the left wall [meters]
  # Outputs the PID error required to make the car follow the left wall.
  def followLeft(self, desired_distance=DESIRED_DISTANCE):
    return self.follow(desired_distance, math.pi) * -1

  # data: single message from topic /scan
  # desired_distance: desired distance to the right wall [meters]
  # Outputs the PID error required to make the car follow the right wall.
  def followRight(self, desired_distance=DESIRED_DISTANCE):
    return self.follow(desired_distance, 0)
    #alpha is returned in radians

  # data: single message from topic /scan
  # Outputs the PID error required to make the car drive in the middle
  # of the hallway.
  def followCenter(self):
    #Get left and right perpendicular distance, add and divide by 2 to get
    #center distance, then follow right wall with desired distance as d_centre

    #get left distance
    b = self.getRange(math.pi/4+math.pi)
    a = self.getRange(math.pi/4 + abs(math.pi-THETA))
    #alpha is returned in radians
    alpha = math.atan((a*math.cos(abs(THETA)) - b)/(a*math.sin(abs(THETA))))
    d_tl = b*math.cos(alpha)

    #get right distance
    b = self.getRange(math.pi/4)
    a = self.getRange(math.pi/4 + abs(THETA))
    #alpha is returned in radians
    alpha = math.atan((a*math.cos(abs(THETA)) - b)/(a*math.sin(abs(THETA))))
    d_tr = b*math.cos(alpha)

    d_centre = (d_tr + d_tl)/2

    d_tplus1 = d_tr + lookDistance*math.sin(alpha)
    error = (d_centre - d_tplus1)
    if np.isnan(error):
      error = 0
    else:
      if np.isinf(error):
     	error = self.lidarScan.range_max

    return error

  def follow(self, desired_distance, angle):
    b = self.getRange(math.pi/4+angle)
    a = self.getRange(math.pi/4 + abs(angle-THETA))
    #alpha is returned in radians
    alpha = math.atan((a*math.cos(abs(THETA)) - b)/(a*math.sin(abs(THETA))))
    d_t = b*math.cos(alpha)
    d_tplus1 = d_t + lookDistance*math.sin(alpha)
    error = (desired_distance - d_tplus1)
    if np.isnan(error):
      error = 0
    else:
      if np.isinf(error):
        error = self.filterScan.range_max
    return error

  def followGap(self, angle):
    return lookDistance*math.sin(angle)

# Boilerplate code to start this ROS node.
# DO NOT MODIFY!
if __name__ == '__main__':
  iface = Interface()
  iface.start()
