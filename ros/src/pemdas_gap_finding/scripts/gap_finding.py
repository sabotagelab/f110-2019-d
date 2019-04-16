#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point
from pemdas_gap_finding.msg import Gaps

#publish all gaps to lidar_gaps
#publish best point to gap_center

class Interface:
    def __init__(self, rate=10):
        rospy.init_node('pemdas_gap_finding')

        self.sub = rospy.Subscriber("/scan", LaserScan, self.callback)
        self.gapPub = rospy.Publisher("/lidar_gaps", Gaps, queue_size=10)
	self.pointPub = rospy.Publisher("/center_point", Point, queue_size=100)
        self.rate = rospy.Rate(rate)
        

    def start(self):
        while not rosy.is_shutdown():
            rospy.spinonce()
            self.rate.sleep()


    def callback(self, scanData):
        gaps = findGaps(scanData.data)
        gapsMessage = makeGapsMessage(gaps, scanData)
        centerPoint = chooseCenterPoint(gaps, scanData)
        centerPointMessage = makeCenterPointMessage(centerPoint)
        self.gapPub.publish(gapsMessage)
        self.pointPub.publish(centerPoint)
	

def findGaps(scanData):
    return []

def makeGapsMessage(gaps, scan):
	msg = Gaps()
	msg.gaps = gaps
	msg.angle_increment = scan.angle_increment

#find center point in global coords from gaps
def chooseCenterPoint(gaps, scan):
	return (0, 0, 0)

def makeCenterPointMessage(centerPoint):
    msg = Point()
    msg.x = centerPoint[0]
    msg.y = centerPoint[1]
    msg.z = centerPoint[2]
    return msg

if __name__ == "__main__":
    try:
        iface = Interface(rate=5)
        iface.start()
    except rospy.ROSInterruptException:
        rospy.logerror("ROS Interrupt Exception")

            

 
