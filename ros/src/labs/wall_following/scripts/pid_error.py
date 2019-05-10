#!/usr/bin/env python
import rospy
import math
import numpy as np
import sys
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64
import pdb
from wall_following.msg import pid_angle_input, follow_type
import yaml
import os
from race.msg import drive_param

pub = rospy.Publisher('pid_error', pid_angle_input, queue_size=10)

dirname = os.path.dirname(__file__)
filepath = os.path.join(dirname, '../config/config.yaml')
with open (filepath, 'r') as f:
	doc = yaml.load(f)

# You can define constants in Python as uppercase global names like these.
MIN_DISTANCE = doc["pid_error"]["MIN_DISTANCE"]
MAX_DISTANCE = doc["pid_error"]["MAX_DISTANCE"]
MIN_ANGLE = 0
MAX_ANGLE = 3*math.pi/2
THETA = math.pi/16

#estimated delay from command to steady state

currentRanges = [1] * 2000
CONTROL_DELAY_ESTIMATE = doc["pid_error"]["CONTROL_DELAY_ESTIMATE"]
lookDistance = doc["pid_error"]["lookDistance"]
DESIRED_DISTANCE = doc["pid_error"]["DESIRED_DISTANCE"]

#historical speed, updated continuosly
lastSpeed = 1

def filterRanges(lidarMessage, coe=1.1):
    from scipy.signal import savgol_filter
    data = np.array(lidarMessage.ranges)
    data[np.isinf(data)] = lidarMessage.range_max * coe
    #data[np.isnan(data)] = lidarMessage.range_max * coe

    nanidx = np.where(np.isnan(data))[0]
    if len(nanidx):
        nanchunks = []
        last = nanidx[0]
        size = 1
        for ri in xrange(1, len(nanidx)):
            if nanidx[ri] - last != 1:
                nanchunks.append((last, size))
                last = nanidx[ri].tolist()
                size = 1
            else:
                size += 1
        if last != None:
            nanchunks.append((last, size))

    
        chunkStart = 0
        for c in nanchunks:
            inc = (data[c[0]-1] - data[c[0]+c[1]-1]) / c[1]
            for i in xrange(chunkStart, chunkStart+c[1]):
                data[nanidx[i]] = data[c[0]-1] + (i-chunkStart) * inc
            chunkStart += c[1]

    data = savgol_filter(data.tolist(), 11, 3)
    return data 

currentMode = doc["pid_error"]["currentMode"]
currentEnumMode = doc["pid_error"]["currentEnumMode"]
currentGapAngle = doc["pid_error"]["currentGapAngle"]
modeMap = {
  "center" : 0,
  "left" : 1,
  "right" : 2,
  "gap" : 3
}

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
  #Get left and right perpendicular distance, add and divide by 2 to get
  #center distance, then follow right wall with desired distance as d_centre

  #get left distance
  b = getRange(data, math.pi/4+math.pi)
  a = getRange(data, math.pi/4 + abs(math.pi-THETA))
  #alpha is returned in radians
  alpha = math.atan((a*math.cos(abs(THETA)) - b)/(a*math.sin(abs(THETA))))
  d_tl = b*math.cos(alpha)

  #get right distance
  b = getRange(data, math.pi/4)
  a = getRange(data, math.pi/4 + abs(THETA))
  #alpha is returned in radians
  alpha = math.atan((a*math.cos(abs(THETA)) - b)/(a*math.sin(abs(THETA))))
  d_tr = b*math.cos(alpha)

  d_centre = (d_tr + d_tl)/2

  d_tplus1 = d_tr + lookDistance*math.sin(alpha)
  error = (d_centre - d_tplus1)
  return error

def follow(data, desired_distance, angle):
  b = getRange(data, math.pi/4+angle)
  a = getRange(data, math.pi/4 + abs(angle-THETA))
  #alpha is returned in radians
  alpha = math.atan((a*math.cos(abs(THETA)) - b)/(a*math.sin(abs(THETA))))
  d_t = b*math.cos(alpha)
  d_tplus1 = d_t + lookDistance*math.sin(alpha)
  error = (desired_distance - d_tplus1)
  if np.isnan(error):
    error = 0
  else:
    if np.isinf(error):
      error = data.range_max
  return error

def followGap(angle):
  return lookDistance*math.sin(angle)

modeEnumMap = dict([
  (0 , followCenter),
  (1 , followLeft),
  (2 , followRight),
  (3 , followGap)
])

# Callback for receiving LIDAR data on the /scan topic.
# data: the LIDAR data, published as a list of distances to the wall.

def scan_callback(data):
  #currentRanges = filterRanges(data)
  error = modeEnumMap[currentEnumMode](data)
#  if error == 0:
#    print(currentRanges)
  msg = pid_angle_input()
  msg.pid_error = error
  msg.header = data.header
  pub.publish(msg)

def estimateLookDistance(data):
  lastSpeed = data
  lookDistance = lastSpeed * lastSpeed

def changeFollowType(data):
  currentEnumMode = data.type
  currentGapAngle = data.gap_angle

def setLookDistance(data):
  lookDistance = data.velocity

# Boilerplate code to start this ROS node.
# DO NOT MODIFY!
if __name__ == '__main__':
  rospy.init_node('pid_error_node', anonymous = True)
  rospy.Subscriber("scan", LaserScan, scan_callback)
  rospy.Subscriber("follow_types", follow_type, changeFollowType)
  rospy.Subscriber("cmd_vel", drive_param, setLookDistance)
  rospy.spin()
