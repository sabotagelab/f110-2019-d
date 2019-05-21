#!/usr/bin/env python
import rospy
from wall_following.msg import pid_angle_input, pid_meta

class Interface:
    def __init__(self):
        rospy.init_node('pid_meta_node', anonymous=True)

        self.errSub = rospy.Subscriber("pid_error", pid_angle_input, self.errAvgCallback)
        self.metaPub = rospy.Publisher("wall_following_analysis", pid_meta, queue_size=5)

        self.pidErrCount = 0
        self.pidErrAvg = 0
        self.pidErrMax = -1000000
        self.pidErrMin = 10000000
    
    def start(self):
        rospy.spin()

    def errAvgCallback(self, data):
        self.pidErrCount += 1

        if data.pid_error > self.pidErrMax:
            self.pidErrMax = data.pid_error
        if data.pid_error < self.pidErrMin:
            self.pidErrMin = data.pid_error

        self.pidErrAvg = (self.pidErrAvg * (self.pidErrCount-1) + data.pid_error) / self.pidErrCount
        
        msg = pid_meta()
        msg.error_avg = self.pidErrAvg
        msg.error_max = self.pidErrMax
        msg.error_min = self.pidErrMin
        self.metaPub.publish(msg)

if __name__ == "__main__":
    iface = Interface()
    iface.start()
