# This script interacts with the vehicle carla object and gets its information which
# is used to form various transformation matrices. It also gives bounding boxes of
# vehicle objects.


import numpy as np
import math
pi = math.pi



def GetHomogeneousTrans(x, y, z, roll, pitch, yaw):
	''' Form homogeneous transformation'''
	# Convert to ROS right handed convention
	R_x = np.array(
		[
			[1, 0, 0],
			[0, math.cos(roll), -math.sin(roll)],
			[0, math.sin(roll), math.cos(roll)],
		]
	)

	R_y = np.array(
		[
			[math.cos(pitch), 0, math.sin(pitch)],
			[0, 1, 0],
			[-math.sin(pitch), 0, math.cos(pitch)],
		]
	)

	R_z = np.array(
		[
			[math.cos(yaw), -math.sin(yaw), 0],
			[math.sin(yaw), math.cos(yaw), 0],
			[0, 0, 1],
		]
	)

	R = np.matmul(R_z, np.matmul(R_y, R_x))

	t = np.array([x, y, z])

	H = np.c_[R, t]
	base = np.array([0, 0, 0, 1])
	H = np.r_[H, [base]]

	return H



def extractActorInfo(vehicle):
	''' This function extracts info such as position and 
	orientation'''
	# Getting position
	x = vehicle.get_transform().location.x
	y = -vehicle.get_transform().location.y
	z = vehicle.get_transform().location.z
	t = np.array([x, y, z])
	# Getting orientation and converting to radians
	roll = -(vehicle.get_transform().rotation.roll)*pi/180
	pitch = -(vehicle.get_transform().rotation.pitch)*pi/180
	yaw = -(vehicle.get_transform().rotation.yaw)*pi/180

	return [x, y, z, roll, pitch, yaw]


def getLocalBB(vehicle):
	'''This function returns the bounding box corners in 
	a local frame'''
	dimX = vehicle.bounding_box.extent.x
	dimY = vehicle.bounding_box.extent.y
	dimZ = vehicle.bounding_box.extent.z
	c1 = [dimX, dimY, dimZ]
	c2 = [dimX, -dimY, dimZ]
	c3 = [dimX, -dimY, -dimZ]
	c4 = [dimX, dimY, -dimZ]

	c5 = [-dimX, dimY, dimZ]
	c6 = [-dimX, -dimY, dimZ]
	c7 = [-dimX, -dimY, -dimZ]
	c8 = [-dimX, dimY, -dimZ]
	# Adding center
	c0 = [0, 0, 0]
	return [c0, c1, c2, c3, c4, c5, c6, c7, c8]


def getCarBBTrans(vehicle):
	'''This function returns the transformation of the
	vehicle bounding box w.r.t world frame'''
	t_x = vehicle.bounding_box.location.x
	t_y = -vehicle.bounding_box.location.y
	t_z = vehicle.bounding_box.location.z
	# Transform to the car frame
	H_Car2BB = np.array([[1, 0, 0, t_x], [0, 1, 0, t_y], [0, 0, 1, t_z], [0, 0, 0, 1]])
	# Transform global
	x, y, z, roll, pitch, yaw = extractActorInfo(vehicle)
	H_W2Car = GetHomogeneousTrans(x, y, z, roll, pitch, yaw)
	H_W2BB = np.matmul(H_W2Car, H_Car2BB)

	return H_W2BB


def getGlobalBB(vehicle):
	'''This function returns the bounding box corners in 
	a global frame'''
	H_W2BB = getCarBBTrans(vehicle)
	localBB = np.asarray(getLocalBB(vehicle)).T
	localBB = np.r_[localBB, [np.ones(9)]]
	globalBB = np.matmul(H_W2BB, localBB)
	# NOTE: returns numpy array 4x9
	return globalBB


def getBBEGo(ego_vehicle, vehicle):
	''' Gives bounding box corners w.r.t ego-vehicle'''
	H_W2Ego = getCarBBTrans(ego_vehicle)
	globalBB = getGlobalBB(vehicle)
	egoBB = np.matmul(np.linalg.pinv(H_W2Ego), globalBB)

	return egoBB


def getBBRADAR(egoBB, RADAR_transform):
	'''This function tranforms the points in the ego-
	vehicle frame to the RADAR frame'''
	x, y, z, roll, pitch, yaw = RADAR_transform
	# Transformation from ego to RADAR
	H_R2Ego = GetHomogeneousTrans(x, y, z, roll, pitch, yaw)
	# Bounding box in RADAR frame
	BBRADAR = np.matmul(H_R2Ego, egoBB)

	return BBRADAR


def getMaxMinBBVals(ego_vehicle, vehicle, RADAR_transform):
	''' Get max vals of bounding box'''
	# NOTE: I have defined y as vertical and x horizontal.
	# NOTE: The return values are w.r.t to the RADAR
	# Carla does it opposite
	# -ve sign is to account for ROS convention
	# Find max y val and corresponding x val
	egoBB = getBBEGo(ego_vehicle, vehicle)
	# Get bounding box in RADAR frame
	BBRADAR = getBBRADAR(egoBB, RADAR_transform)
	y = BBRADAR[1, 0]
	x = BBRADAR[0, 0]
	ind_argMax = np.argmax(BBRADAR[1, :])
	ind_argMax = np.argmin(BBRADAR[1, :])
	# Switching x and y
	y_max = BBRADAR[1, ind_argMax]
	x_max = BBRADAR[0, ind_argMax]
	# Similarly for min
	y_min = BBRADAR[1, ind_argMax]
	x_min = BBRADAR[0, ind_argMax]

	return [x, y, x_max, y_max, x_min, y_min]
