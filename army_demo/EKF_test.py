#!/usr/bin/env python

import numpy as np
import matplotlib.pyplot as plt

def main():

	wheel_odom = np.genfromtxt("./wheel_odom.csv",delimiter=',')
	gps_odom = np.genfromtxt("./gps_only.csv",delimiter=',')
	wheel_odom = wheel_odom[1:]
	gps_odom = gps_odom[1:]
	states = []
	x = np.array([0,0,0])
	u = np.array([0,0])
	z = np.array([0,0])
	dt = 0.1
	#F = np.eye(2)
	P = np.eye(3)*0.01
	H = np.array([[1,0,0],
				  [0,1,0]])
	R = np.eye(2)*7
	sigma_u = np.eye(2)*0.1
	#G = np.eye(2)*dt

	counter = 0

	states.append(x)

	for i in range(1010):

		u[0]=wheel_odom[i][48]
		u[1]=wheel_odom[i][53]
		#u[1]=wheel_odom[i][49]
		
		z[0]=gps_odom[counter][5]
		z[1]=gps_odom[counter][6]

		if i % 10 == 0:

			temp_x = np.zeros((3))

			temp_x[0] = x[0] + u[0] * (np.cos(x[2])) * dt
			temp_x[1] = x[1] + u[0] * (np.sin(x[2])) * dt
			temp_x[2] = x[2] + u[1] * dt

			F = np.array([[1, 0, -u[0]*np.sin(x[2])*dt],  
			              [0, 1,  u[0]*np.cos(x[2])*dt], 
			              [0, 0, 1]])

			G = np.array([[np.cos(x[2]) * dt, 0],
			              [np.sin(x[2]) * dt, 0],
			              [0, dt]])

			Q = np.dot(G, sigma_u)
			Q = np.dot(Q, np.transpose(G))

			temp_p = np.dot(F, P)
			temp_p = np.dot(temp_p, np.transpose(F))
			temp_p = temp_p + Q

			temp_z = np.dot(H, np.transpose(x))

			K = np.dot(temp_p, np.transpose(H))
			temp = np.dot(H, temp_p)
			temp = np.dot(temp, np.transpose(H))
			temp = np.linalg.pinv(temp + R)
			K = np.dot(K, temp)

			P = np.dot((np.eye(3) - np.dot(K, H)), temp_p)
			x = temp_x + np.dot(K, (z - temp_z))
			"""
			temp_x = x + (u * dt)
			Q = np.dot(G, sigma_u)
			Q = np.dot(Q, np.transpose(G))

			temp_cov = np.dot(F, P)
			temp_cov = np.dot(temp_cov, np.transpose(F))
			temp_cov = temp_cov + Q

			temp_z = np.dot(H, np.transpose(x))
			K = np.dot(temp_cov, np.transpose(H))
			temp = np.dot(H, temp_cov)
			temp = np.dot(temp, np.transpose(H))
			temp = np.linalg.pinv(temp + R)
			K = np.dot(K, temp)

			P = np.dot((np.eye(2) - np.dot(K,H)), temp_cov)
			x = temp_x + np.dot(K, (z - temp_z))
			"""
			counter += 1

		else:
			x[0] = x[0] + u[0] * (np.cos(x[2])) * dt
			x[1] = x[1] + u[0] * (np.sin(x[2])) * dt
			x[2] = x[2] + u[1] * dt

		states.append(x)

	states = np.array(states)
	print(states.shape)
	plt.xlim(-10,20)
	plt.ylim(-10,20)
	plt.scatter(states[:,0],states[:,1],s=1)
	#plt.plot(states[:,2])
	plt.show()


if __name__ == '__main__':
	main()