#!/usr/bin/env python

import rospy
from race.msg import drive_param
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker
from nav_msgs.msg import Odometry
import math
import numpy as np
import numpy.ma as ma
from numpy import linalg as LA
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import tf
import csv
import os
import time

class Interface:

    def __init__(self):
        rospy.init_node('pure_pursuit')

        self.LOOKAHEAD_DISTANCE = .75 #meters
        self.VELOCITY = 1.0 #m/s
        self.FOV = 120 #where to look for new points
        self.FOV_MULT = 1 / np.tan( np.deg2rad(self.FOV) / 2 ) # constant for calculating critical x-value
        self.WHEELBASE = .326 #meters
        self.MAX_TURN_ANGLE = .4189

        self.LOCALIZATION_DELAY = .1 #lag from localization in seconds

        #EXPERIMENTAL VELOCITY SETTINGS
        self.MAX_VELOCITY = 2.0 #m/s
        self.MIN_VELOCITY = 1.0 #m/s
        self.VELOCITY_ANGLE_RELATION = (self.MAX_VELOCITY-self.MIN_VELOCITY) / self.MAX_TURN_ANGLE
        self.VELOCITY_ANGLE_RELATION_SQR = (self.MAX_VELOCITY**2-self.MIN_VELOCITY**2) / self.MAX_TURN_ANGLE

        self.waypointFilepath = '/home/nvidia/rcws/logs/test-path.csv'

        self.poseSub = rospy.Subscriber('/pf/viz/inferred_pose', PoseStamped, self.localizeCallback, queue_size=1)
        self.odomSub = rospy.Subscriber('/vesc/odom', Odometry, self.storeOdometry, queue_size=5)
        self.drivePub = rospy.Publisher('drive_parameters', drive_param, queue_size=1)
        self.waypointPub = rospy.Publisher('/waypoint_marker', Marker)
        self.tfListener = tf.TransformListener()

    def start(self):
        rospy.spin()
    
    def storeOdometry(self, odom):
        self.currentVelocity = (odom.pose.twist.twist.linear.x, odom.pose.twist.twist.linear.y)
        self.currentSpeed = np.linalg.norm(self.currentVelocity)

    def loadWaypoints(self):

        dirname = os.path.dirname(__file__)
        filename = os.path.join(dirname, '~/rcws/logs/test-path.csv')
        with open(self.waypointFilepath) as f:
            path_points = [tuple(line) for line in csv.reader(f)]

        # Turn path_points into a list of floats to eliminate the need for casts in the code below.
        self.path_points = [[float(point[0]), float(point[1])] for point in path_points]
        print(path_points)
        # Publisher for 'drive_parameters' (speed and steering angle)

    def localizeCallback(self, data):
        mapToLaser = self.tfListener.lookupTransform('/map','/laser',rospy.Time(0))
        
        toLaserTransMatrix = np.asarray(mapToLaser[0][:2])

        carOrientQTMap = (
            data.pose.orientation.x,
            data.pose.orientation.y,
            data.pose.orientation.z,
            data.pose.orientation.w
        )

        carYawMap = euler_from_quaternion(carOrientQTMap)[2] #yaw

        toLaserRotMatrix = np.array([
            [ np.cos(carYawMap), -np.sin(carYawMap) ],
            [ np.sin(carYawMap), np.cos(carYawMap) ]
        ])

        toLaser = lambda coords : (coords - toLaserTransMatrix).dot(toLaserRotMatrix)

        carPositionMap = np.array([
            data.pose.position.x,
            data.pose.position.y
        ])

        carPositionLaser = toLaser(carPositionMap) #this should be the zero vector
        #apply positional extrapolation based on localization delay
        carPositionLaser += self.extrapolatePosition() 

        pathPointsLaser = toLaser(np.asarray(self.path_points))

        distances = np.linalg.norm(pathPointsLaser - carPositionLaser, axis=1)
        mask = np.ones(len(distances), dtype=int)
        mask[np.where(np.logical_and(
            (distances >= self.LOOKAHEAD_DISTANCE), 
            (pathPointsLaser[:,0] > (self.FOV_MULT * pathPointsLaser[:,1]))
        )] = 0
        viable = ma.masked_array(distances, mask=mask)
        waypointIndex = viable.argmin()
        waypoint = pathPointsLaser[waypointIndex]

        goalX = waypoint[0]
        goalY = waypoint[1]
    
        self.turnRadius = distances[waypointIndex]**2 / (2 * np.abs(goalY))
        self.curvature = 1 / self.turnRadius
        
        self.steeringAngle = np.arcsin(self.WHEELBASE/self.turnRadius) * np.sign(goalY)
        self.steeringAngle = np.clip(self.steeringAngle, -self.MAX_TURN_ANGLE, self.MAX_TURN_ANGLE)
        
        msg = drive_param()
        msg.velocity = self.decideVelocity()
        msg.angle = self.steeringAngle
        self.drivePub.publish(msg)

    def extrapolatePosition(self):
        theta = self.currentSpeed * self.LOCALIZATION_DELAY * self.curvature
        return np.array([
            self.turnRadius * np.cos(theta),
            self.turnRadius * np.sin(theta)
        ]) * np.sign(self.steeringAngle)

    def decideVelocity(self):
        return self.VELOCITY
        #linear speed reduction on corners
        #return self.MIN_VELOCITY + self.VELOCITY_ANGLE_RELATION * (self.MAX_TURN_ANGLE - self.steeringAngle) 

        #basic polynomial reduction
        #return self.MIN_VELOCITY + self.VELOCITY_ANGLE_RELATION_SQR * (self.MAX_TURN_ANGLE - self.steeringAngle)

        #force-based polynomial reduction

    def publishMarker(self):
        pass
        #marker = Marker()
        #marker.header.frame_id = "/map"
        #marker.type = marker.SPHERE
        #marker.action = marker.ADD
        #marker.scale.x = 0.3
        #marker.scale.y = 0.3
        #marker.scale.z = 0.3
        #marker.color.a = 0.0
        #marker.color.r = 1.0
        #marker.color.g = 0.0
        #marker.color.b = 0.0
        #marker.pose.orientation.w = 1.0
        #marker.pose.position.x = goalX + toLaserFrame[0]
        #marker.pose.position.y = goalY + toLaserFrame[1]
        #marker.pose.position.z = 0
        #self.waypointPub.publish(marker)

if __name__ == "__main__":
    try:
        iface = Interface()
        iface.start()
    except rospy.ROSInterruptException:
        rospy.logerr("ROS INTERRUPT EXCEPTION")
