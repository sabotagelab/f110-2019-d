#!/usr/bin/env python

from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Vector3
from pemdas_gap_finding.msg import PointArray
import rospy
import colorsys as cs

# We will publish Marker type messages to this topic. When opening Rviz, we select this topic for visualization (just as we select topic /scan, say) and see the markers
publisher = rospy.Publisher('/visualization_gaps_all', MarkerArray, queue_size="1")

# HSVA colors
centerColor = [1.0, 1.0, 1.0, 1.0]
edgeColor = [1.0, 0.5, 1.0, 1.0]

maxColorGaps = 4.0

# Input data is custom Gaps msg representing sides and centers of all gaps from scan
def callback(data):
    #while not rospy.is_shutdown():
    markerArray = MarkerArray()
    markerArray.markers = [] 
    for pi in range(len(data.points)):
        point = data.points[pi]
        is_center = True if (pi - 1) % 3 == 0 else False
        color = centerColor if is_center else edgeColor
        color = [c for c in color]
        
        if pi < maxColorGaps * 3:
            color[0] = (float(int(pi /3) + 1)/maxColorGaps) * color[0]
            print(color[0])
        
        color = list(cs.hsv_to_rgb(color[0], color[1], color[2]))
        print(color)
        color.append(1.0)

        marker = Marker()

    # Specify the frame in which to interpret the x,y,z coordinates. It is the laser frame.
        marker.id = pi + 10
        marker.header.frame_id = "/laser"
        marker.pose.position.x = point.x
        marker.pose.position.y = point.y
        marker.pose.position.z = point.z # or set this to 0

        marker.type = marker.SPHERE

        marker.scale.x = 1.0 - (not is_center) * .5# If marker is too small in Rviz can make it bigger here
        marker.scale.y = 1.0 - (not is_center) * .5
        marker.scale.z = 1.0 - (not is_center) * .5
        marker.color.r = color[0] 
        marker.color.g = color[1] 
        marker.color.b = color[2] 
        marker.color.a = color[3]

        markerArray.markers.append(marker)

    # Publish the MarkerArray
    print("Sending marker array")
    publisher.publish(markerArray)

if __name__ == '__main__':
    rospy.init_node('visualize_gaps_all')
    rospy.Subscriber('/lidar_gap_points', PointArray, callback)
    rospy.spin()