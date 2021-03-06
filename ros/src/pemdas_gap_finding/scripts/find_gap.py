#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point
from pemdas_gap_finding.msg import Gaps, Gap, LidarPoint
import tf
import numpy as np
from collections import deque
import time

import sys, os
sys.path.append(os.path.join(os.path.dirname(sys.path[0]), 'src'))
from algorithms import processGaps, globalizePoint, gradientScan_np, gradientScan_nan

PUBLISH_GAP_POINTS = False
if PUBLISH_GAP_POINTS:
    from pemdas_gap_finding.msg import PointArray

class Interface:
    def __init__(self, rate=10, window=10):
        rospy.init_node('pemdas_gap_finding')

        self.sub = rospy.Subscriber("/scan", LaserScan, self.callback)
        #self.subTF = rospy.Subscriber("/tf", TFMessage, self.storeTF)
        #self.subTF_static = rospy.Subscriber("/tf_static", TFMessage, self.storeTF)

        self.gapPub = rospy.Publisher("/lidar_gaps", Gaps, queue_size=100)
        self.pointPub = rospy.Publisher("/gap_center", Point, queue_size=100)
        if PUBLISH_GAP_POINTS:
            self.gapPointsPub = rospy.Publisher("/lidar_gap_points", PointArray, queue_size=5)

        self.rate = rospy.Rate(rate)

        self.tfListener = tf.TransformListener()

        self.lidarWindow = deque(maxlen=window)

        makeProbability = lambda x : pow(10, -(window-x))
        self.lidarHistoryGradient = np.asarray(list(map(makeProbability, range(0, window))))
        print(self.lidarHistoryGradient)

    def start(self):
        rospy.spin()


    def callback(self, lidarData):
        #rospy.loginfo("Recieved filtered lidar data")
        #kSeed = find_k(scanData)
        #kSeedNP = find_k_np(scanData)
        #rospy.loginfo("Found k seed value: %i" % kSeed)
        #rospy.loginfo("Found np k seed value: %i" % kSeedNP)

        #we only want to pop once the window is full
        if not len(self.lidarWindow) < self.lidarWindow.maxlen:
            self.lidarWindow.pop()
        self.lidarWindow.appendleft(lidarData)
        #avgData = np.average(np.asarray(self.lidarWindow), weights=self.lidarHistoryGradient)


        gaps = gradientScan_nan(lidarData)

        scores, linearDistances, centerGap = processGaps(gaps)

        try:
            #transferQT = self.tfListener.lookupTransform('/map', '/laser', rospy.Time(0))

            if PUBLISH_GAP_POINTS:
                features = lambda gap : [gap[0], gap[int(len(gap) / 2)], gap[-1]]
                gapPoints = PointArray()
                gapPoints.points = []
                for gap in gaps:
                    for point in features(gap):
                        gapPoints.points.append(Point(*globalizePoint(fixAngle(point, scanData), *transferQT)))
                self.gapPointsPub.publish(gapPoints)

            centerPoint = fixAngle(centerGap[int(len(centerGap) / 2)], lidarData)
            centerPoint = globalizePoint(centerPoint)
            centerPointMessage = Point(*centerPoint)
            self.pointPub.publish(centerPointMessage)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logerr("Exception transforming centerpoint. Will not publish to /center_point")
            rospy.logerr(e)

        gapsMessage = makeGapsMessage(gaps, linearDistances, scores)

        self.gapPub.publish(gapsMessage)

        #rospy.loginfo("Published Gapfinding Info")
        #self.rate.sleep()

def fixAngle(point, scanData):
    angleRangeHalf = (scanData.angle_max - scanData.angle_min) / 2
    #center = (gap[1][0] * .25, gap[1][1] + angleRangeHalf + 3.14)
    center = (point[0] , point[1] - angleRangeHalf)

    return center

#DEPRECATED - FOR USE WITH KMEANS
#def makeGapsMessage(gaps, linearDistances):
    #rospy.loginfo(linearDistances)
    #lidarPoints = [LidarPoint(*point) for gap in gaps for point in gap]

    #n = 3
    #gapGroup = lambda idx, sep : lidarPoints[ (idx * sep) : (idx * sep) + sep ]
    #msg = [ Gap(linearDistances[i], gapGroup(i, n)) for i in xrange(0, len(gaps))]

    #return msg

def makeGapsMessage(gaps, linearDistances, scores):
    features = lambda gap : [gap[0], gap[int(len(gap) / 2)], gap[-1]]
    lidarPoints = [LidarPoint(*point) for gap in gaps for point in features(gap)]

    n = 3
    gapGroup = lambda idx, sep : lidarPoints[ (idx * sep) : (idx * sep) + sep ]
    features = [Gap([LidarPoint(*f) for f in features(gap)]) for gap in gaps]
    return Gaps(features, linearDistances, scores)

if __name__ == "__main__":
    try:
        iface = Interface(rate=5)
        iface.start()
    except rospy.ROSInterruptException:
        rospy.logerr("ROS Interrupt Exception")
