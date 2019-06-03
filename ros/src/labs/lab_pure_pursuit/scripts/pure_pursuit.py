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
import yaml


dirname = os.path.dirname(__file__)
filepath = os.path.join(dirname, '../config/' + configFile)

with open (filepath, 'r') as f:
  doc = yaml.load(f)
  doc = doc["pure_pursuit"]


class Interface:

    def __init__(self):
        rospy.init_node('pure_pursuit')

        self.FOV = doc["FOV"] #where to look for new points
        self.FOV_MULT = 1 / np.tan( np.deg2rad(self.FOV) / 2 ) # constant for calculating critical x-value
        self.LOOKAHEAD_DISTANCE = doc["LOOKAHEAD_DISTANCE"] #m
        self.WHEELBASE = doc["WHEELBASE"] #meters
        self.MAX_TURN_ANGLE = doc["MAX_TURN_ANGLE"]
        self.TURN_ANGLE_NORMALIZATION = 1/self.MAX_TURN_ANGLE

        self.LOCALIZATION_DELAY = doc["LOCALIZATION_DELAY"] #lag from localization in seconds

        self.MAX_VELOCITY = doc["MAX_DISTANCE"] #m/s
        self.MIN_VELOCITY = doc["MIN_VELOCITY"] #m/s
        self.VELOCITY_NORMALIZATION = 1/(self.MAX_VELOCITY - self.MIN_VELOCITY)
        self.VELOCITY_ANGLE_RELATION = (self.MAX_VELOCITY-self.MIN_VELOCITY) / self.MAX_TURN_ANGLE
        self.VELOCITY_ANGLE_RELATION_SQR = (self.MAX_VELOCITY**2-self.MIN_VELOCITY**2) / self.MAX_TURN_ANGLE

        self.ANGLE_LOOK_RATIO = doc["ANGLE_LOOK_RATIO"]
        self.MAX_LOOKAHEAD = doc["MAX_LOOKAHEAD"] #m
        self.MIN_LOOKAHEAD = doc["MIN_LOOKAHEAD"] #m
        self.SPD_LOOKAHEAD_EXPONENT = doc["SPD_LOOKAHEAD_EXPONENT"]
        self.LOOK_SPEED_RELATION = (self.MAX_LOOKAHEAD - self.MIN_LOOKAHEAD) / (self.MAX_VELOCITY - self.MIN_VELOCITY)
        self.getLookAtSpeed = lambda : (self.MAX_LOOKAHEAD - self.MIN_LOOKAHEAD) * \
            ((np.clip(self.currentSpeed - self.MIN_VELOCITY,0,self.MAX_VELOCITY)
             * self.VELOCITY_NORMALIZATION)**self.SPD_LOOKAHEAD_EXPONENT) \
            * (( 1 - self.ANGLE_LOOK_RATIO) + self.ANGLE_LOOK_RATIO * \
            (1 - np.abs(self.steeringAngle * self.TURN_ANGLE_NORMALIZATION))) \
            + self.MIN_LOOKAHEAD

        self.MIN_META_LOOKAHEAD = doc["MIN_META_LOOKAHEAD"] #m
        self.MAX_META_LOOKAHEAD = doc["MAX_META_LOOKAHEAD"] #m

        self.ANGLE_TURN_EXPONENT = doc["ANGLE_TURN_EXPONENT"] # how quickly speed changes with low angles (lower numbers are less agressive)
        self.getSpeedAtAngle = lambda : (self.MAX_VELOCITY - self.MIN_VELOCITY) * \
            ( 1 - np.abs(self.steeringAngle * self.TURN_ANGLE_NORMALIZATION)**self.ANGLE_TURN_EXPONENT ) \
            + self.MIN_VELOCITY

        self.waypointFilepath = '/home/nvidia/rcws/logs/test-path.csv'

        self.poseSub = rospy.Subscriber('/pf/viz/inferred_pose', PoseStamped, self.localizeCallback, queue_size=1)
        self.odomSub = rospy.Subscriber('/vesc/odom', Odometry, self.storeOdometry, queue_size=1)
        self.drivePub = rospy.Publisher('drive_parameters', drive_param, queue_size=1)
        self.waypointPub = rospy.Publisher('/waypoint_marker', Marker, queue_size=10)
        self.tfListener = tf.TransformListener()

        self.path_points = None
        self.loadWaypoints()

        self.curvature = 1
        self.steeringAngle = 0
        self.turnRadius = 1
        self.currentVelocity = np.array([0,0])
        self.currentSpeed = 0

    def start(self):
        rospy.spin()

    def storeOdometry(self, odom):
        self.currentVelocity = np.array([odom.twist.twist.linear.x, odom.twist.twist.linear.y])
        self.currentSpeed = np.linalg.norm(self.currentVelocity)

    def loadWaypoints(self):

        dirname = os.path.dirname(__file__)
        filename = os.path.join(dirname, '~/rcws/logs/test-path.csv')
        with open(self.waypointFilepath) as f:
            self.path_points = [tuple(line) for line in csv.reader(f)]

        # Turn path_points into a list of floats to eliminate the need for casts in the code below.
        self.path_points = [[float(point[0]), float(point[1])] for point in self.path_points]
        print(self.path_points)

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

        #finding goal waypoint and other metadata
        carPositionMap = np.array([
            data.pose.position.x,
            data.pose.position.y
        ])

        carPositionLaser = toLaser(carPositionMap) #this should be the zero vector

        #apply positional extrapolation based on localization delay
        extrap = self.extrapolatePosition()

        pathPointsLaser = toLaser(np.asarray(self.path_points))

        distances = np.linalg.norm(pathPointsLaser - carPositionLaser, axis=1)
        mask = np.ones(len(distances), dtype=int)
        viable = np.where(np.logical_and(
            (distances >= self.LOOKAHEAD_DISTANCE),
            (pathPointsLaser[:,0] > (self.FOV_MULT * pathPointsLaser[:,1]))
        ))
        mask[viable] = 0

        viableMask = ma.masked_array(distances, mask=mask)
        waypointIndex = viableMask.argmin()
        waypoint = pathPointsLaser[waypointIndex]

        goalX = waypoint[0]
        goalY = waypoint[1]

        #calculate goal angle and velocity
        self.findCurveRadius = lambda distance, offset : distance**2/(2 * np.abs(offset))
        self.turnRadius = self.findCurveRadius(distances[waypointIndex], goalY)
        self.curvature = 1 / self.turnRadius

        self.steeringAngle = np.arcsin(self.WHEELBASE/self.turnRadius) * np.sign(goalY)
        self.steeringAngle = np.clip(self.steeringAngle, -self.MAX_TURN_ANGLE, self.MAX_TURN_ANGLE)

        print("===========================")
        print("X: " + str(goalX))
        print("Y: " + str(goalY))
        print("R: " + str(self.turnRadius))
        print("angle: " + str(self.steeringAngle))
        print("ex: " + str(extrap[0]))
        print("ey: " + str(extrap[1]))
        print("LOOK: " + str(self.decideLookahead()))
        print("SPD: " + str(self.decideVelocity()))
        print("SPD-EST: " + str(self.currentSpeed))
        print("===========================")


        msg = drive_param()
        msg.velocity = self.decideVelocity()
        msg.angle = self.steeringAngle
        self.drivePub.publish(msg)

    def extrapolatePosition(self):
        theta = self.currentSpeed * self.LOCALIZATION_DELAY * self.curvature
        print(theta)
        return np.array([
            self.turnRadius * np.cos(theta),
            self.turnRadius * np.sin(theta)
        ]) * np.sign(self.steeringAngle)

    def decideVelocity(self):
        #return self.VELOCITY
        #linear speed reduction on corners
        #return self.MIN_VELOCITY + self.VELOCITY_ANGLE_RELATION * (self.MAX_TURN_ANGLE - self.steeringAngle)

        #basic polynomial reduction
        #return self.MIN_VELOCITY + self.VELOCITY_ANGLE_RELATION_SQR * (self.MAX_TURN_ANGLE - self.steeringAngle)

        return self.getSpeedAtAngle()

    def decideLookahead(self):
        #return np.clip(self.currentSpeed - self.MIN_VELOCITY, 0, self.MAX_VELOCITY) * self.LOOK_SPEED_RELATION + self.MIN_LOOKAHEAD
        return self.getLookAtSpeed()

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
