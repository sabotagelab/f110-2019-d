#!/usr/bin/env python

import rospy
from race.msg import drive_param
from std_msgs.msg import Float64
from wall_following.msg import pid_angle_input
from collections import deque 
import numpy as np
import math


# TODO: modify these constants to make the car follow walls smoothly.
KP = .4
KI = .0006
KD = .6

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

		self.drivePub = rospy.Publisher('drive_parameters', drive_param, queue_size=10)
		self.errSub = rospy.Subscriber("pid_error", pid_angle_input, self.control_callback)

		self.lastErr = 0
		self.calls = 0
		self.errTotal = 0
		self.angle = 0

	def start(self):
		rospy.spin()
	
	def derivativeError(self):
		err = np.asarray(self.errorWindow)
		return np.average(np.gradient(err), weights=self.weights)
	
	def angleMaxVelocity(self, angle):
		#avgAngle = abs(np.average(np.asarray(self.angleWindow))) #sign does not matter since we are only determining speed
		# if abs(angle) > SPD_DEC_ANGLE_MAX:
		# 	setang = SPD_DEC_ANGLE_MAX

		# vel = MAX_VEL * (1-abs(setang)/SPD_DEC_ANGLE_MAX)
		# if vel < MIN_VEL:
		# 	vel = MIN_VEL	
		if (0 < abs(angle) and abs(angle) < np.deg2rad(10)):
			vel = 1.5
		else:
			if (np.deg2rad(10) <= abs(angle) and abs(angle) < np.deg2rad(20)):
				vel = 1.0
			else:
				vel = .5

		return vel


		# avgAngle = max(min(avgAngle, SPD_DEC_ANGLE_MAX), 0) #clamp angle between 0 and max angle
		# angleStepDecrease = (int(avgAngle / SPD_DEC_ANGLE_PERIOD)) * A
		# angleRemainderInc = angleLimitFunc(np.rad2deg(avgAngle % SPD_DEC_ANGLE_PERIOD))
		# return MAX_VEL - MIN_VEL - angleStepDecrease + angleRemainderInc

	def control_callback(self, data):
	# TODO: Based on the error (data.data), determine the car's required velocity
	# and steering angle.
		et = data.pid_error
		print(et)

		self.calls += 1
		self.errTotal += et


		deriverr = (et - self.lastErr)
		interr = self.errTotal

		ut = KP * et + KI * interr + KD * deriverr
		self.angle += ut
		if (abs(self.angle) > SPD_DEC_ANGLE_MAX or np.isnan(self.angle)):
			self.angle = np.sign(self.angle)*SPD_DEC_ANGLE_MAX
		
		self.lastErr = et

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

