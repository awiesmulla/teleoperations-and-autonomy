#!/usr/bin/env python

import rospy
import numpy as np

from nav_msgs.msg import Odometry

class EKF:
	
	def __init__(self):
		
		self.pub = rospy.Publisher("pose", Odometry, queue_size=10)

		self.dt = 0.1
		self.state = np.zeros(2)
		self.u = np.zeros(2)
		self.z = np.zeros(2)
		self.G = np.eye(2) * self.dt
		self.sigma_u = np.eye(2) * 0.1
		self.F = np.eye(2)
		self.P = np.eye(2) * 0.01
		self.H = np.eye(2)
		self.R = np.eye(2) * 7


	def filter(self):

		self.gps = rospy.Subscriber("gps_odom", Odometry, self.correction)
		self.odom = rospy.Subscriber("odometry/filtered", Odometry, self.measurement)

	def measurement(self,msg):

		self.state[0] = msg.pose.pose.x
		self.state[1] = msg.pose.pose.y
		self.u[0] = msg.twist.linear.x
		self.u[1] = msg.twist.linear.y
		self.pub.publish(msg)

	def gps(self,msg):

		self.z[0] = msg.pose.pose.x
		self.z[1] = msg.pose.pose.y
		self.fuse()

	def fuse(self):

		x = self.state + (self.u * self.dt)

		Q = np.dot(self.G, self.sigma_u)
		Q = np.dot(Q, np.transpose(self.G))

		temp_cov = np.dot(self.F, self.P)
		temp_cov = np.dot(temp_cov, np.transpose(self.F))
		temp_cov = temp_cov + Q

		temp_z = np.dot(self.H, np.transpose(self.state))

		K = np.dot(temp_cov, np.transpose(self.H))
		temp = np.dot(self.H, temp_cov)
        temp = np.dot(temp, np.transpose(self.H))
        temp = np.linalg.pinv(temp + self.R)
        K = np.dot(K, temp)

        self.P = np.dot((np.eye(3) - np.dot(K,self.H)), temp_cov)
        self.state = x + np.dot(K, (self.z - temp_z))

        msg = Odometry()
        msg.pose.pose.x = self.state[0]
        msg.pose.pose.y = self.state[1]
        msg.twist.linear.x = self.u[0]
        msg.twist.linear.y = self.u[1]
        self.pub.publish(msg)


if __name__ == '__main__':
	
	rospy.init_node('EKF')
	ekf = EKF()
	ekf.filter()
	rospy.spin()