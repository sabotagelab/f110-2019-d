#!/usr/bin/env python

import rospy
from race.msg import drive_param
from std_msgs.msg import Float64
from wall_following.msg import pid_angle_input
from collections import deque 
import numpy as np
import math


# TODO: modify these constants to make the car follow walls smoothly.
KP = 1.7
KI = .001
KD = 1

N = 1
K = .5
weightFunc = lambda x : N*math.exp(-K*x)

SPD_DEC_ANGLE_PERIOD = np.deg2rad(10)
SPD_DEC_ANGLE_MAX = np.deg2rad(20)
MAX_VEL = 1.5 #m/s
MIN_VEL = .5

A = .5 #top of decrease
B = 0 #bottom of decrease
C = 8 #centerpoint of decrease
R = 2 #steepness of decrease (negative values make increase)
angleLimitFunc = lambda angle : (A*math.exp(C*R) + B * math.exp(R*angle))/(math.exp(C*R) + math.exp(R*angle))

# Callback for receiving PID error data on the /pid_error topic
# data: the PID error from pid_error_node, published as a Float64
class Interface:

	def __init__(self, errWindowLen, angleWindowLen):
		rospy.init_node('pid_controller_node', anonymous=True)

		self.drivePub = rospy.Publisher('drive_parameters', drive_param, queue_size=1)
		self.errSub = rospy.Subscriber("pid_error", pid_angle_input, self.control_callback)

		self.errorWindow = deque([0] * errWindowLen, maxlen=errWindowLen)
		self.etInt = 0
		self.weights = np.asarray(map(weightFunc, range(errWindowLen)))

		self.angleWindow = deque([0] * angleWindowLen, maxlen=angleWindowLen)
		self.lastTime = 0
		self.currentTime = 0

		self.angle = 0
		self.lastAngle = 0
		self.lastError = 0

	def start(self):
		rospy.spin()

	def storeQ(self, q, elem):
		if not len(q) < q.maxlen:
			q.pop()
		q.appendleft(elem)
	
	def storeError(self, elem):
		self.storeQ(self.errorWindow, elem)
	
	def storeAngle(self, elem):
		self.storeQ(self.angleWindow, elem)
	
	def derivativeError(self, error):
		return (error - self.lastError)
		#err = np.asarray(self.errorWindow)
		#return np.average(np.gradient(err), weights=self.weights)
	
	def integralError(self, error):
		return error#(self.currentTime - self.lastTime) * error
	
	def angleMaxVelocity(self, angle):
		#avgAngle = abs(np.average(np.asarray(self.angleWindow))) #sign does not matter since we are only determining speed
		avgAngle = angle
		avgAngle = max(min(avgAngle, SPD_DEC_ANGLE_MAX), 0) #clamp angle between 0 and max angle
		angleStepDecrease = (int(avgAngle / SPD_DEC_ANGLE_PERIOD)) * A
		angleRemainderInc = angleLimitFunc(np.rad2deg(avgAngle % SPD_DEC_ANGLE_PERIOD))
		return MAX_VEL - MIN_VEL - angleStepDecrease + angleRemainderInc

	def control_callback(self, data):
		#calculate frame time
		self.lastTime = self.currentTime
		self.currentTime = float(data.header.stamp.nsecs) * pow(10, -9)

		#get and store error
		et = data.pid_error
		self.storeError(et)

		#weighted pid equation for angle increment
		ut = KP * et + KI * self.integralError(et) + KD * self.derivativeError(et)
		self.angle += ut
		if np.isnan(self.angle):
			self.angle = np.nanmean(self.angleWindow)
		self.angle = max(min(self.angle, SPD_DEC_ANGLE_MAX), -1*SPD_DEC_ANGLE_MAX) #clamp angle between -/+ max angle

		#store historical data
		self.lastError = et
		self.storeAngle(self.angle)

		msg = drive_param()
		msg.angle = self.angle    # TODO: implement PID for steering angle
		msg.velocity = self.angleMaxVelocity(self.angle)  # TODO: implement PID for velocity
		print("ANGLE: ", np.rad2deg(self.angle))
		print("VEL: ", msg.velocity)
		self.drivePub.publish(msg)

# Boilerplate code to start this ROS node.
# DO NOT MODIFY!
if __name__ == '__main__':
	iface = Interface(10, 5)
	iface.start()

