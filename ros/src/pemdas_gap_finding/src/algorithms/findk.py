#!/usr/bin/env python
import rospy
import math
import numpy as np
from std_msgs.msg import String
#import matplotlib.pyplot
from sensor_msgs.msg import LaserScan

def callback(data):
    #print(data.ranges)
    k = find_k(data.ranges)
    rospy.loginfo(rospy.get_caller_id() + "K = %i", k)

def find_k(scan):
    #Set inf and nan values to a value greater than the furthest scan
    scanData = scan
    scan = scan.ranges
    coe = 1.2
    scan = [(scanData.range_max * coe) if math.isnan(x) else x for x in scan]
    scan = [(scanData.range_max * coe) if math.isinf(x) else x for x in scan]
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
    std  = math.sqrt(var)
    thres = mean + z*std
    hits = []
    last = 0
    for i in range(len(ddscan)):
        if (ddscan[i] > thres):
            hits.append(i)
            if i-last>1:
                k = k+1
            last = i
    
    return k

def find_k_np(scan, z=2, coe=1.1):
    data = scan.ranges
    data = np.array(data)
    data[np.isinf(data)] = scan.range_max * coe
    data[np.isnan(data)] = scan.range_max * coe

    diffs = np.absolute(np.diff(data))

    mean = np.mean(data)
    std = np.std(data)
    threshhold = mean + z * std

    hits = np.where(data > threshhold)
    hitsShift = np.append(hits[1:], np.nan)

    ki = hits[hits != hitsShift]
    k = len(ki)
    return k

def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("/scan", LaserScan , callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()