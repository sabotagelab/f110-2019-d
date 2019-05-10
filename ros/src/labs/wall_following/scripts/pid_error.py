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


dirname = os.path.dirname(__file__)
filepath = os.path.join(dirname, '../config/config.yaml')
with open (filepath, 'r') as f:
  doc = yaml.load(f)
  doc = doc["pid_error"]

# You can define constants in Python as uppercase global names like these.
MIN_DISTANCE = doc["MIN_DISTANCE"]
MAX_DISTANCE = doc["MAX_DISTANCE"]
MIN_ANGLE = 0
MAX_ANGLE = 3*math.pi/2
THETA = math.pi/16

#estimated delay from command to steady state

CONTROL_DELAY_ESTIMATE = doc["CONTROL_DELAY_ESTIMATE"]
lookDistance = doc["lookDistance"]
DESIRED_DISTANCE = doc["DESIRED_DISTANCE"]

#historical speed, updated continuosly
lastSpeed = 1

currentMode = doc["currentMode"]
currentEnumMode = doc["currentEnumMode"]
currentGapAngle = doc["currentGapAngle"]
modeMap = {
  "center" : 0,
  "left" : 1,
  "right" : 2,
  "gap" : 3
}

class Interface:
  def __init__(self):
    rospy.init_node('pid_error_node', anonymous = True)
    self.pidErrorPub = rospy.Publisher('pid_error', pid_angle_input, queue_size=10)
    self.laserScanSub = rospy.Subscriber("scan", LaserScan, self.scan_callback)
    self.followTypePub = rospy.Subscriber("follow_types", follow_type, self.changeFollowType)
    self.cmdVelSub = rospy.Subscriber("cmd_vel", drive_param, self.setLookDistance)
  
    self.currentRanges = [1] * 2000

    self.modeEnumMap = dict([
      (0 , self.followCenter),
      (1 , self.followLeft),
      (2 , self.followRight),
      (3 , self.followGap)
    ])

  def start(self):
    rospy.spin()

  # Callback for receiving LIDAR data on the /scan topic.
  # data: the LIDAR data, published as a list of distances to the wall.
  def scan_callback(self, data):
    self.lidarScan = data
    self.filterScan = self.filterRanges()
    error = self.modeEnumMap[currentEnumMode]()
  #  if error == 0:
  #    print(currentRanges)
    msg = pid_angle_input()
    msg.pid_error = error
    msg.header = data.header
    self.pidErrorPub.publish(msg)

  def changeFollowType(self, data):
    currentEnumMode = data.type
    currentGapAngle = data.gap_angle

  def setLookDistance(self, data):
    lookDistance = data.velocity

  # data: single message from topic /scan
  # angle: between 0(far right) to 270 (far left) degrees, where 45 degrees is directly to the right
  # Outputs length in meters to object with angle in lidar scan field of view
  def getRange(self, angle):
    inc = self.lidarScan.angle_increment
    index = int(angle / inc)
    index = np.clip(index, 0, len(self.filterScan)-1)
    return self.filterScan[index]

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
        error = self.lidarScan.range_max
    return error

  def followGap(self, angle):
    return lookDistance*math.sin(angle)

  def filterRanges(self, coe=1.1):
      from scipy.signal import savgol_filter
      data = np.array(self.lidarScan.ranges)
      data[np.isinf(data)] = self.lidarScan.range_max * coe
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
              inc = (data[c[0]-1] - data[c[0]+c[1]]-1) / c[1]
              for i in xrange(chunkStart, chunkStart+c[1]):
                  data[nanidx[i]] = data[c[0]-1] + (i-chunkStart) * inc
              chunkStart += c[1]

      data = savgol_filter(data.tolist(), 11, 3)
      return data

# Boilerplate code to start this ROS node.
# DO NOT MODIFY!
if __name__ == '__main__':
  iface = Interface()
  iface.start()
