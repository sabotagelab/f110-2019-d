#!/usr/bin/env python

import rospy
from race.msg import drive_param
from std_msgs.msg import Float64
from wall_following.msg import pid_angle_input
from collections import queue
import numpy as np


# TODO: modify these constants to make the car follow walls smoothly.
KP = 0.0
KI = 0.0
KD = 0.0

N = 1
K = .5
weightFunc = lambda x : N*math.exp(-K*x)
angleLimitFunc = lambda a : (a*math.exp(c*r) + b * math.exp(r*a))/(math.exp(c*r) + math.exp(r*a))

# Callback for receiving PID error data on the /pid_error topic
# data: the PID error from pid_error_node, published as a Float64
class Interface:

	def __init__(self, errWindowLen):
		rospy.init_node('pid_controller_node', anonymous=True)

		self.drivePub = rospy.Publisher('drive_parameters', drive_param, queue_size=1)
		self.errSub = rospy.Subscriber("pid_error", pid_angle_input, control_callback)

		self.errorWindow = queue(maxlen=errWindowLen)
		self.etInt = 0
		self.weights = np.asarray(map(weightFunc), xrange(errWindowLen))

		self.angleWindow = queue(maxlen=angleWindowLen)

	def start(self):
		rospy.spin()

	def storeQ(self, q, elem):
		if not q.maxlen < len(q):
			q.pop()
		q.appendleft(elem)
	
	def storeError(self, elem):
		storeQ(self.errWindow, elem)
	
	def storeAngle(self, elem):
		storeQ(self.angleWindow, elem)
	
	def derivativeError(self):
		err = np.asarray(self.errorWindow)
		return np.average(np.gradient(err), weights=self.weights)
	
	def angleMaxVelocity(self):
		return np.average(np.asarray(self.angleWindow))

	def control_callback(data):
	# TODO: Based on the error (data.data), determine the car's required velocity
	# amd steering angle.
		et = data.error
		storeError(et)

		currentTime = float(data.header.nsecs) * pow(10, -9)
		self.etInt += et * (currentTime - self.lastTime)
		self.lastTime = currentTime

		ut = KP * et + KI * self.etInt + KD * derivativeError()

		msg = drive_param()
		msg.angle = ut    # TODO: implement PID for steering angle
		msg.velocity = self.angleMaxVelocity(ut)  # TODO: implement PID for velocity
		pub.publish(msg)

# Boilerplate code to start this ROS node.
# DO NOT MODIFY!
if __name__ == '__main__':
	iface = Interface(0, 10, 5)
	iface.start()

