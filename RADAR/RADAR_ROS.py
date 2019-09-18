# This script mimics a RADAR sensor. It request all its transformations from the ClientRADAR script.
# Refer that script for all transformations and bounding box values of all vehicles in world or ego
# vehicle frame


import numpy as np
import math
import matplotlib.pyplot as plt

import ClientRADAR_ROS as CRD_ROS

class RADAR:
	''' This class instantiates a RADAR object'''

	def __init__(self, thetaR, range):
		''' thetaR is the angular range of the RADAR.
		range is the distance up to which RADAR detects objects'''
		self.thetaR = thetaR
		self.range = range


class Vehicle:
	''' instantiates a vehicle object. It has x,y of centroid and
	bounding box with points (x_max, y_max) and (x_min,y_min)'''

	# NOTE: y_max is not actually max y co-ordinate and similarly for y_min
	# NOTE: Get actual vehicle ID to add as an attribute
	def __init__(self, id, x, y, x_max, y_max, x_min, y_min, velocity):
		self.id = id
		self.x = x
		self.y = y
		self.x_max = x_max  # box looks like [[x_max,y],[x_min,y]]
		self.y_max = y_max
		self.x_min = x_min
		self.y_min = y_min
		self.velocity = velocity
		self.polygon = [
			[self.x, self.y, 0],
			[self.x_max, self.y_max, 0],
			[self.x_min, self.y_min, 0],
		]


def RADAR_FOV(env_vehicles, RADAR):
	'''Given a list of vehicles in the environment. This function filters all
	those vehicles that are not in the field of view(FOV) of the RADAR.
	It returns a list of vehicles (x,y,yaw) of all vehicles in FOV'''
	# Iterate over all y values of vehicles in
	FOV_vehicles = []
	# print("Printing list of vehicles in env")
	for vehicle in env_vehicles:
		# Take the left most point of the vehicle and take abs
		x = vehicle.x_min
		y = vehicle.y_min
		# print(vehicle.x, vehicle.y)
		if x >= 1 / math.tan(RADAR.thetaR / 2) * abs(y) and x <= RADAR.range and x > 0:
			FOV_vehicles.append(vehicle)
	return FOV_vehicles


def RADAR_detected(env_vehicles, RADAR):
	'''Returns list of vehicle objects detected by RADAR'''

	FOV_vehicles = RADAR_FOV(env_vehicles, RADAR)
	if len(FOV_vehicles) == 0:
		print("None in FOV")
		return []
		# Sort vehicles based on x values

	FOV_vehicles_sorted = sorted(FOV_vehicles, key=lambda x: x.x)
	# print("Number of objects sorted:")
	# print(len(FOV_vehicles_sorted))
	# print("Proceeding")

	# For every detected vehicle my RADAR will not be able to detect the
	# vehicles other detected vehicles are blocking. Every equation will have a
	# slope
	slopes = []
	detected_vehicles = []
	flag = False
	# Iterate through FOV vehicles based already sorted based on x
	for vehicle in FOV_vehicles_sorted:
		x_max = vehicle.x_max
		x_min = vehicle.x_min
		y_max = vehicle.y_max
		y_min = vehicle.y_min
		# Find slopes of the detected vehicle
		m1 = math.tan(x_max / y_max)
		m2 = math.tan(x_min / y_min)
		if vehicle == FOV_vehicles_sorted[0]:
			slopes.append([m1, m2])
			detected_vehicles.append(vehicle)
		else:
			for m in slopes:
				if m1 < m[0] or m2 > m[1]:
					flag = True
				else:
					flag = False
		if flag == True:
			slopes.append([m1, m2])
			detected_vehicles.append(vehicle)
			# Adding noise
	addRADARNoise(detected_vehicles)
	for vehicle_radar in detected_vehicles:
		print(vehicle_radar.x, vehicle_radar.y)
	return detected_vehicles


def addRADARNoise(detected_vehicles):
	n = 0.8  # change this value for noises within 30m range
	m = 1.3  # change this value for noises after 30m range
	# TODO: Add noise in velocity
	for vehicle in detected_vehicles:
		if vehicle.x < 30:
			noise = np.random.rand() * 2 * n - n
			vehicle.y_max = vehicle.y_max + noise
			vehicle.y_min = vehicle.y_min - noise
			vehicle.velocity[0] = vehicle.velocity[0] + noise/2
			vehicle.velocity[1] = vehicle.velocity[0] + noise

		else:
			noise = np.random.rand() * 2 * m - m
			vehicle.y_max = vehicle.y_max + noise
			vehicle.y_min = vehicle.y_min - noise
			vehicle.velocity[0] = vehicle.velocity[0] + noise/2
			vehicle.velocity[1] = vehicle.velocity[0] + noise




def getVelocity_ROS(ego_vehicle, vehicle):
	'''Returns velocity with some added noise'''
	H_W2Ego = CRD_ROS.getCarBBTrans(ego_vehicle)
	x_vel = vehicle.get_velocity().x
	y_vel = -vehicle.get_velocity().y
	z_vel = vehicle.get_velocity().z
	# The above values are in the world coordinate frame
	# Transforming them w.r.t ego vehicle frame
	vel_vec = np.array([x_vel, y_vel, z_vel, 0])
	vel_ego = np.matmul(np.linalg.pinv(H_W2Ego), vel_vec)
	vel_ego = vel_ego.tolist()

	return vel_ego


# This is the final function that needs to run in order to visualize
# a RADAR output

def simulateRADAR(theta, range, actor_list, ego_vehicle, RADAR_transform):
	''' Simulate and visualize RADAR output'''
	delta_radar = RADAR(theta, range)
	env_vehicles = []
	# Iterate over vehicles in actor list and create vehicle class objects
	for obj in actor_list:
		id = obj.id
		velocity_ego = getVelocity_ROS(ego_vehicle, obj)
		x, y, x_max, y_max, x_min, y_min = CRD_ROS.getMaxMinBBVals(ego_vehicle, obj, RADAR_transform)
		# All these values are w.r.t the center of the ego vehicle.
		# Tranform these values to the RADAR position
		pose = np.array([[x, y, 0, 1], [x_max, y_max, 0, 1], [x_min, y_min, 0, 1]])
		car = Vehicle(id, x, y, x_max, y_max, x_min, y_min, velocity_ego)
		env_vehicles.append(car)
  
	# id = ego_vehicle.id
	# velocity_ego = getVelocity_ROS(ego_vehicle, ego_vehicle)
	# x, y, x_max, y_max, x_min, y_min = CRD_ROS.getMaxMinBBVals(ego_vehicle, ego_vehicle, RADAR_transform)
	# Truck = Vehicle(id, x, y, x_max, y_max, x_min, y_min, velocity_ego)
	# print(env_vehicles)
	detected_RADAR = RADAR_detected(env_vehicles, delta_radar)
	# visualizeRADAR(env_vehicles, detected_RADAR, delta_radar, Truck)

	return detected_RADAR

def visualizeRADAR(env_vehicles, detected_vehicles, RADAR, Truck):
	'''This function shows a scatter plot of vehicles in environment,
	the RADAR detected vehicles, and the ego vehicle'''
	env_plot = [[env_v.x, env_v.y] for env_v in env_vehicles]
	env_plot = np.asarray(env_plot)

	det_plot = [[det_v.x, det_v.y] for det_v in detected_vehicles]
	det_plot = np.asarray(det_plot)

	fig = plt.figure()
	ax1 = fig.add_subplot(111)

	ax1.scatter(
		env_plot[:, 0], env_plot[:, 1], s=10, c='b', marker="s", label='All vehicles'
	)
	if len(detected_vehicles) != 0:
		ax1.scatter(
			det_plot[:, 0], det_plot[:, 1], s=10, c='r', marker="o", label='Detected'
		)
	ax1.scatter(Truck.x, Truck.y, s=15, c='g', marker="+", label='Ego Vehicle')
	# Plot RADAR FOV
	xlim = math.tan(RADAR.thetaR / 2) * RADAR.range
	x = np.linspace(-xlim, xlim, 100)
	y = 1 / math.tan(RADAR.thetaR / 2) * abs(x)
	ax1.plot(x, y, '-r', label='FOV')
	# Set limits for plot
	ax1.set_xlim(-100, 100)
	ax1.set_ylim(-500, 500)
	plt.legend(loc='upper left')
	plt.show()


