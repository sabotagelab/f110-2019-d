#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point
from tf2_msgs.msg import TFMessage
from pemdas_gap_finding.msg import Gaps, Gap, LidarPoint
import tf

#from gap_finding_algorithms import findGaps, processGaps, globalizePoint

#publish all gaps to lidar_gaps
#publish best point to gap_center

class Interface:
    def __init__(self, rate=10):
        rospy.init_node('pemdas_gap_finding')

        self.sub = rospy.Subscriber("/scan", LaserScan, self.callback)
        self.tfListener = tf.TransformListener()

        self.gapPub = rospy.Publisher("/lidar_gaps", Gaps, queue_size=10)
        self.pointPub = rospy.Publisher("/center_point", Point, queue_size=100)

        self.rate = rospy.Rate(rate)
        

    def start(self):
        rospy.spin()


    def callback(self, scanData):
        gaps = findGaps(scanData)
        linearDistances, centerGap = processGaps(gaps, scanData)

        try:
            transferQT = self.tfListener.lookupTransform('/laser', '/map', rospy.Time(0))
            centerPoint = globalizePoint(centerGap[1], *transferQT)
            centerPointMessage = makeCenterPointMessage(centerPoint)
            self.pointPub.publish(centerPointMessage)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logerror("Exception transforming centerpoint. Not publishing to /center_point")

        gapsMessage = makeGapsMessage(gaps, linearDistances)

        self.gapPub.publish(gapsMessage)

        rospy.loginfo("Published Gapfinding Info")
        self.rate.sleep()

def findGaps(scanData):
    #k = findk(scanData)
    #return kmeans(scanData, k=k)
    # return structure - [ [ (range, angle), (..), (..) ], [...], [...], ...]

def globalizePoint(center, trans, rot):
    return [0, 0, 0]

def makeGapsMessage(gaps, linearDistances):
    lidarPoints = [LidarPoint(*point) for gap in gaps for point in gap]

    n = 3
    gapGroup = lambda idx, sep : points[ (idx * sep) : (idx * sep) + sep ]
    msg = [ Gap(*gapGroup(i, n), linearDistances[i]) for i in xrange(0, len(lidarPoints))]

    return msg

def makeCenterPointMessage(centerPoint):
    msg = Point(*centerPoint)
    return msg

if __name__ == "__main__":
    try:
        iface = Interface(rate=5)
        iface.start()
    except rospy.ROSInterruptException:
        rospy.logerror("ROS Interrupt Exception")

            

 
