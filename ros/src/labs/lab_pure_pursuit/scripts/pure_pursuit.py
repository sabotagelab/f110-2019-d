#!/usr/bin/env python

import rospy
from race.msg import drive_param
from geometry_msgs.msg import PoseStamped
import math
import numpy as np
import numpy.ma as ma
from numpy import linalg as LA
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import tf
import csv
import os
import time

#############
# CONSTANTS #
#############

LOOKAHEAD_DISTANCE = .5 # meters
VELOCITY = 0.5 # m/s
FOV = 120

tfListener = None

###########
# GLOBALS #
###########

# Import waypoints.csv into a list (path_points)
dirname = os.path.dirname(__file__)
filename = os.path.join(dirname, '~/rcws/logs/test-path.csv')
filepath = '/home/nvidia/rcws/logs/test-path.csv'
with open(filepath) as f:
    path_points = [tuple(line) for line in csv.reader(f)]

# Turn path_points into a list of floats to eliminate the need for casts in the code below.
path_points = [[float(point[0]), float(point[1])] for point in path_points]
print(path_points)
# Publisher for 'drive_parameters' (speed and steering angle)
pub = rospy.Publisher('drive_parameters', drive_param, queue_size=1)


#############
# FUNCTIONS #
#############

# Computes the Euclidean distance between two 2D points p1 and p2.
def dist(p1, p2):
    return np.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)

# Input data is PoseStamped message from topic /pf/viz/inferred_pose.
# Runs pure pursuit and publishes velocity and steering angle.
def callback(data):

    # Note: These following numbered steps below are taken from R. Craig Coulter's paper on pure pursuit.

    # 1. Determine the current location of the vehicle (we are subscribed to vesc/odom)
    # Hint: Read up on PoseStamped message type in ROS to determine how to extract x, y, and yaw.

    #rospy.init_node('pure_pursuit_node', anonymous=True)
    #drivePub = rospy.Publisher('drive_parameters', drive_param, queue_size=1)
    #PoseSub = rospy.Subscriber("/vesc/odom", PoseStamped, self.callback)

    mapToLaser = tfListener.lookupTransform('/map','/laser',rospy.Time(0))
    toLaserFrame = np.asarray(mapToLaser[0][:2])

    pathPointsLaser = np.asarray(path_points) - toLaserFrame
    #get x, y and yaw
    carPositionWorld = np.array([ data.pose.position.x, data.pose.position.y ])

    quaternion = (
        data.pose.orientation.x,
        data.pose.orientation.y,
        data.pose.orientation.z,
        data.pose.orientation.w)

    euler = euler_from_quaternion(quaternion)
    yaw = euler[2]


    # 2. Find the path point closest to the vehicle that is >= 1 lookahead distance from vehicle's current location.
    d = np.linalg.norm(pathPointsLaser - (carPositionWorld-toLaserFrame), axis=1)
    print(d[::10])
    mask = np.ones(len(d), dtype=int)
    mask[np.where(np.logical_or((d >= LOOKAHEAD_DISTANCE), (pathPointsLaser[:,0] < 0)))] = 0
    viable = ma.masked_array(d, mask=mask)
    waypoint = pathPointsLaser[viable.argmin()]

    goalX = waypoint[0]
    goalY = waypoint[1]
    angle = np.arctan(goalY/goalX)


    #    print("CURRENT POS")
    #    print("\t X: " + str(x))
    #    print("\t Y: " + str(y))
    #    print("\t facing: " + str(yaw))

    #    print("WAYPOINT")
    #    print("\t X: " + str(goalX))
    #    print("\t Y: " + str(goalY))
    print("\t at angle: " + str(angle))
    #path_points[index] is the point closest to the vehicle

    # 3. Transform the goal point to vehicle coordinates.
    # THIS IS DONE IN LOOP

    # 4. Calculate the curvature = 1/r = 2x/l^2
    # The curvature is transformed into steering wheel angle by the vehicle on board controller.
    # Hint: You may need to flip to negative because for the VESC a right steering angle has a negative value.


    angle = np.clip(angle, -0.4189, 0.4189) # 0.4189 radians = 24 degrees because car can only turn 24 degrees max

    msg = drive_param()
    msg.velocity = VELOCITY
    msg.angle = -1*angle
    pub.publish(msg)

if __name__ == '__main__':
    rospy.init_node('pure_pursuit')
    rospy.Subscriber('/pf/viz/inferred_pose', PoseStamped, callback, queue_size=1)
    tfListener = tf.TransformListener()
    rospy.spin()
