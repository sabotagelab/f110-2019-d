#!/usr/bin/env python
import rospy
from race.msg import drive_param
from std_msgs.msg import Float64
from wall_following.msg import pid_angle_input, follow_type
from collections import deque
import numpy as np
import math
import time
import yaml
import os
import sys

configFile = "config.yaml"
if len(sys.argv) > 1:
    configFile = sys.argv[1]
dirname = os.path.dirname(__file__)
filepath = os.path.join(dirname, '../config/' + configFile)
#print(filepath)

with open (filepath, 'r') as f:
    doc = yaml.load(f)
    doc = doc["control"]

KP = doc["KP"]
KI = doc["KI"]
KD = doc["KD"]

KP_TM = doc["KP_TM"] if "KP_TM" in doc else 1
KI_TM = doc["KI_TM"] if "KI_TM" in doc else 1
KD_TM = doc["KD_TM"] if "KD_TM" in doc else 1

N = doc["N"]
K = doc["K"]
weightFunc = lambda x : N*math.exp(-K*x)

SPD_DEC_ANGLE_PERIOD = np.deg2rad(10)
SPD_DEC_ANGLE_MAX = np.deg2rad(doc["MAX_STEERING_ANGLE"] if "MAX_STEERING_ANGLE" in doc else 20)
MAX_VEL = doc["MAX_VEL"] #m/s
MIN_VEL = doc["MIN_VEL"]


A = .5 #top of decrease
B = 0 #bottom of decrease
C = 8 #centerpoint of decrease
R = 1.5 #steepness of decrease (negative values make increase)
angleLimitFunc = lambda angle : (A*math.exp(C*R) + B * math.exp(R*angle))/(math.exp(C*R) + math.exp(R*angle))

# Callback for receiving PID error data on the /pid_error topic
# data: the PID error from pid_error_node, published as a Float64
class Interface:

    def __init__(self, errWindowLen, angleWindowLen):
        rospy.init_node('pid_controller_node', anonymous=True)

        self.drivePub = rospy.Publisher('drive_parameters', drive_param, queue_size=1)
        self.errSub = rospy.Subscriber("pid_error", pid_angle_input, self.control_callback)

        self.followSub = rospy.Subscriber("follow_type", follow_type, self.setTurnMode)

        self.errorWindow = deque([0] * errWindowLen, maxlen=errWindowLen)
        self.etInt = 0
        self.weights = np.asarray(map(weightFunc, range(errWindowLen)))

        self.angleWindow = deque([0] * angleWindowLen, maxlen=angleWindowLen)
        self.lastTime = time.time()
        self.currentTime = 0

        self.angle = 0
        self.lastAngle = 0
        self.lastError = 0
        self.turning = False

        self.velocityMultiple = 1
        if len(sys.argv) > 2 and sys.argv[2] == "true":
            self.velocityMultiple = -1

    def start(self):
        rospy.spin()

    def storeQ(self, q, elem):
        if not len(q) < q.maxlen:
            q.pop()
        q.appendleft(elem)
    
    def setTurnMode(self, data):
        self.turning = data.turnMode

    def storeError(self, elem):
        self.storeQ(self.errorWindow, elem)

    def storeAngle(self, elem):
        self.storeQ(self.angleWindow, elem)

    def derivativeError(self, error):
        return (error - self.lastError) / (self.currentTime - self.lastTime) *  KD_TM
        #err = np.asarray(self.errorWindow)
        #return np.average(np.gradient(err), weights=self.weights)

    def integralError(self, error):
        return (self.currentTime - self.lastTime) * error * KI_TM

    def proportionError(self, error):
        return error * KP_TM

    def angleMaxVelocity(self, angle):
        #avgAngle = abs(np.average(np.asarray(self.angleWindow))) #sign does not matter since we are only determining speed
        avgAngle = min(abs(angle), SPD_DEC_ANGLE_MAX)
        #angleStepDecrease = (int(avgAngle / SPD_DEC_ANGLE_PERIOD)) * A
        #angleRemainderInc = angleLimitFunc(np.rad2deg(avgAngle % SPD_DEC_ANGLE_PERIOD))
        #return MAX_VEL - MIN_VEL - angleStepDecrease + angleRemainderInc
        return (SPD_DEC_ANGLE_MAX - avgAngle)/(SPD_DEC_ANGLE_MAX)*(MAX_VEL-MIN_VEL) + MIN_VEL

    def control_callback(self, data):
        #calculate frame time
        self.currentTime = time.time()#float(data.header.stamp.nsecs) * pow(10, -9)
        #print(self.currentTime - self.lastTime)

        #get and store error
        et = data.pid_error
        self.storeError(et)

        self.etInt += self.integralError(et)

        #weighted pid equation for angle increment
        ut = KP * (et) + KI * self.etInt + KD * self.derivativeError(et)
        self.angle += ut #* (self.currentTime - self.lastTime)
        if np.isnan(self.angle):
            self.angle = np.nanmean(self.angleWindow)
        self.angle = max(min(self.angle, SPD_DEC_ANGLE_MAX), -1*SPD_DEC_ANGLE_MAX) #clamp angle between -/+ max angle

        #store historical data
        self.lastError = et
        self.storeAngle(self.angle)
        self.lastTime = self.currentTime

        msg = drive_param()
        msg.angle = self.angle    # TODO: implement PID for steering angle
        msg.velocity = self.angleMaxVelocity(self.angle) * self.velocityMultiple # TODO: implement PID for velocity
        print("ERROR: ", et)
        print("ANGLE: ", np.rad2deg(self.angle))
        #print("VEL: ", msg.velocity)
        self.drivePub.publish(msg)

# Boilerplate code to start this ROS node.
# DO NOT MODIFY!
if __name__ == '__main__':
    iface = Interface(10, 5)
    iface.start()
