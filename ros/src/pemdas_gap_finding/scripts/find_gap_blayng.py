#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point
from pemdas_gap_finding.msg import Gaps, Gap, LidarPoint
import tf
import numpy as np
from scipy.signal import savgol_filter
import math
# import sys, os
# sys.path.append(os.path.join(os.path.dirname(sys.path[0]), 'src'))
# from algorithms import findGaps, processGaps, globalizePoint, find_k, find_k_np

PUBLISH_GAP_POINTS = True
if PUBLISH_GAP_POINTS:
    from pemdas_gap_finding.msg import PointArray

class Interface:
    def __init__(self, rate=10):
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
        

    def start(self):
        rospy.spin()


    def callback(self, scanData):
        # rospy.loginfo("Recieved Scan Data")
        cent = findCenters(scanData)
        #kSeedNP = find_k_np(scanData)
        # rospy.loginfo("Center found at: %f" % cent)
        #rospy.loginfo("Found np k seed value: %i" % kSeedNP)
        # gaps = findGaps(scanData, k=kSeed)
        # linearDistances, centerGap = processGaps(gaps)


        try:
            transferQT = self.tfListener.lookupTransform('/map', '/laser', rospy.Time(0))

            # if PUBLISH_GAP_POINTS:
            #     gapPoints = PointArray()
            #     gapPoints.points = []
            #     for gap in gaps:
            #         for point in gap:
            #             gapPoints.points.append(Point(*globalizePoint(fixAngle(point, scanData), *transferQT)))
            #     self.gapPointsPub.publish(gapPoints)

            # centerPoint = fixAngle(centerGap[1], scanData)
            # centerPoint = globalizePoint(centerPoint, *transferQT)
            centerPoint = polar2cart(cent)
            centerPointMessage = Point()
            centerPointMessage.x = centerPoint[0]
            centerPointMessage.y = centerPoint[1]
            centerPointMessage.z = centerPoint[2]

            self.pointPub.publish(centerPointMessage)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logerr("Exception transforming centerpoint. Will not publish to /center_point")
            rospy.logerr(e)

        # gapsMessage = makeGapsMessage(gaps, linearDistances)

        # self.gapPub.publish(gapsMessage)

        rospy.loginfo("Published Gapfinding Info")
        self.rate.sleep()

def fixAngle(point, scanData):
    angleRangeHalf = (scanData.angle_max - scanData.angle_min) / 2
    #center = (gap[1][0] * .25, gap[1][1] + angleRangeHalf + 3.14)
    center = (point[0] , point[1] + angleRangeHalf)

    # print('CENTER:',center)
    return center

def polar2cart(best):
    x = best[0] * np.cos(best[1])
    y = best[0] * np.sin(best[1])
    z = 0
    return (x,y,z)

def findCenters(data):
    #Set inf and nan values to a value greater than the furthest scan
    scan = savgol_filter(data.ranges,11,3)
    coe = 1.1
    scan = [(data.range_max * coe) if math.isnan(x) else x for x in scan]
    scan = [(data.range_max * coe) if math.isinf(x) else x for x in scan]
    k = 1
    dscan = []
    ddscan = []

    #Find difference between subsequent elements in scan
    for i in range(len(scan)-1):
        dscan.append(scan[i+1]-scan[i])
        if i > 0:
            ddscan.append(abs(dscan[i]-dscan[i-1])) 
    z = 2
    mean = sum(ddscan) / len(ddscan)   # mean
    var  = sum(pow(x-mean,2) for x in ddscan) / len(ddscan)  # variance
    std  = np.sqrt(var)
    thres = mean + z*std
    hits = []
    last = 0
    for i in range(len(ddscan)):
        if (ddscan[i] > thres):
            hits.append(i)
            if i-last>1:
                k = k+1
            last = i
    hits.append(len(scan)-1)
    spike = []
    feat = [0]
    for i in range(len(hits)-1):
        if hits[i+1]-hits[i]==1:
            spike.append(hits[i])
        else:
            spike.append(hits[i])
            feat.append(int(round((spike[0]+spike[-1])/2)))
            spike = []

    feat.append(len(scan)-1)
    bestGap = [0,0]
    for i in range(len(feat)-1):
        newb = np.average(scan[feat[i]:feat[i+1]])
        if newb>bestGap[0]:
            dist = np.average(scan[feat[i]:feat[i+1]])
            ang = (round((feat[i+1]+feat[i])/2)*data.angle_increment)
            ang -= (data.angle_max-data.angle_min)/2
            bestGap = [dist , ang]
    return bestGap

def makeGapsMessage(gaps, linearDistances):
    rospy.loginfo(linearDistances)
    lidarPoints = [LidarPoint(*point) for gap in gaps for point in gap]

    n = 3   
    gapGroup = lambda idx, sep : lidarPoints[ (idx * sep) : (idx * sep) + sep ]
    msg = [ Gap(linearDistances[i], gapGroup(i, n)) for i in xrange(0, len(gaps))]

    return msg

if __name__ == "__main__":
    try:
        iface = Interface(rate=5)
        iface.start()
    except rospy.ROSInterruptException:
        rospy.logerror("ROS Interrupt Exception")