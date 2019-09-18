#!/usr/bin/env python
# -*- coding: utf-8 -*-

'''
Author  : Prateek Parmeshwar
Email   : pparmesh@andrew.cmu.edu
Date    : Apr 20, 2019
'''

import rospy
import tf
import math
import numpy as np

from nav_msgs.msg import Odometry

# Initialize Globals
ODOMETRY = []
T_INIT = 0
T_END = 0
DISTANCE = 0
XVEL, YVEL = 0, 0

def odometry_callback(odom):
	xPos = odom.pose.pose.position.x
	yPos = odom.pose.pose.position.y
	# Append these to global
	global ODOMETRY
	ODOMETRY.append([xPos, yPos])


def data_callback(odom):
	global T_INIT, T_END, DISTANCE, XVEL, YVEL

	vel = math.sqrt(XVEL**2 + YVEL**2)
	T_END = rospy.get_time()
	DISTANCE += vel*(T_END - T_INIT)
	T_INIT = T_END

	XVEL, YVEL = odom.twist.twist.linear.x, odom.twist.twist.linear.y

	print(DISTANCE)




def dataCollection(mode):
	rospy.init_node('data_collection', anonymous=True)
	
	global T_INIT 
	T_INIT = rospy.get_time()

	if mode == 'validator':
		rospy.Subscriber("/carla/ego_vehicle/odometry", Odometry, odometry_callback)

	if mode == 'CollectData':
		rospy.Subscriber("/carla/ego_vehicle/odometry", Odometry, data_callback)		

	rospy.spin()


if __name__ == '__main__':
	# Mode can be data collection or prediction validator
	# Select validator for data collection for validating ego-prediction
	# Select CollectData to collect data
	mode = 'CollectData'
	dataCollection(mode)
	global ODOMETRY
	ODOMETRY = np.asarray(ODOMETRY)
	np.save('odometry_data', ODOMETRY)
