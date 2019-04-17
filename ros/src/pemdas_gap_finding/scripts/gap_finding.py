#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point
from pemdas_gap_finding.msg import Gaps, Gap, LidarPoint
import tf

import sys
from algorithms import findGaps, processGaps, globalizePoint

#from tf2_msgs.msg import TFMessage

#publish all gaps to lidar_gaps
#publish best point to gap_center

class Interface:
    def __init__(self, rate=10):
        rospy.init_node('pemdas_gap_finding')

        self.sub = rospy.Subscriber("/scan", LaserScan, self.callback)
        #self.subTF = rospy.Subscriber("/tf", TFMessage, self.storeTF)
        #self.subTF_static = rospy.Subscriber("/tf_static", TFMessage, self.storeTF)

        self.gapPub = rospy.Publisher("/lidar_gaps", Gaps, queue_size=100)
        self.pointPub = rospy.Publisher("/center_point", Point, queue_size=100)

        self.rate = rospy.Rate(rate)

        self.tfListener = tf.TransformListener()
        

    def start(self):
        rospy.spin()


    def callback(self, scanData):
        rospy.loginfo("Recieved Scan Data")
        gaps = findGaps(scanData)
        linearDistances, centerGap = processGaps(gaps)

        try:
            transferQT = self.tfListener.lookupTransform('/laser', '/map', rospy.Time(0))
            centerPoint = globalizePoint(centerGap[1], *transferQT)
            centerPointMessage = makeCenterPointMessage(centerPoint)
            self.pointPub.publish(centerPointMessage)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logerr("Exception transforming centerpoint. Will not publish to /center_point")
            rospy.logerr(e)

        gapsMessage = makeGapsMessage(gaps, linearDistances)
        print(gapsMessage)
        self.gapPub.publish(gapsMessage)

        rospy.loginfo("Published Gapfinding Info")
        self.rate.sleep()

#def findGaps(scanData):
    #k = findk(scanData)
    #return kmeans(scanData, k=k)
    # return structure - [ [ (range, angle), (..), (..) ], [...], [...], ...]

#def globalizePoint(center, trans, rot):
#    return [0, 0, 0]

def makeGapsMessage(gaps, linearDistances):
    rospy.loginfo(linearDistances)
    lidarPoints = [LidarPoint(*point) for gap in gaps for point in gap]

    n = 3
    gapGroup = lambda idx, sep : lidarPoints[ (idx * sep) : (idx * sep) + sep ]
    msg = [ Gap(linearDistances[i], gapGroup(i, n)) for i in xrange(0, len(gaps))]

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

            

 
