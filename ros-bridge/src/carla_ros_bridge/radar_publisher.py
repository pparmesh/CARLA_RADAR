#!/usr/bin/env python
# -*- coding: utf-8 -*-

'''
Author  : Prateek Parmeshwar
Email   : pparmesh@andrew.cmu.edu
Version : 1.0.0
Date    : Apr 08, 2019
'''

import carla

from carla_ros_bridge.bridge import CarlaRosBridge
from carla_ros_bridge.bridge_with_rosbag import CarlaRosBridgeWithBag


import random
import threading

import rospy
from geometry_msgs.msg import Point, Polygon, Vector3

from radar_msgs.msg import RadarTrack, RadarTrackArray

import RADAR_ROS as RD_ROS
import math

def list_to_ros_vector3(input_list):
    output_vector = Vector3()
    output_vector.x = input_list[0]
    output_vector.y = input_list[1]
    output_vector.z = input_list[2]
    return output_vector

def polygon_list_to_ros_points(polygon_list):
    output_polygon = Polygon()
    for vertex in polygon_list:
        P = Point()
        P.x = vertex[0]
        P.y = vertex[1]
        P.z = vertex[2]
        output_polygon.points.append(P)
    return output_polygon

def publisher(actor_list, ego_vehicle):
    # global ACTOR_LIST, EGO_VEHICLE

    # Setup node
    pub = rospy.Publisher('/delta/radar/tracks', RadarTrackArray, queue_size=10)
    # rospy.init_node('radar_publisher', anonymous=True)

    # Publish at a rate of 13Hz. This is the RADAR frequency
    r = rospy.Rate(13)

    # Randomly publish some data
    while not rospy.is_shutdown():
        msg = RadarTrackArray()

        # Define RADAR parameters
        thetaR = 2 * math.pi / 3
        rangeR = 100
        radar_transform = [0,0,0,0,0,0]

        # Get list of all detected vehicles

        det_cars = []

        det_cars = RD_ROS.simulateRADAR(thetaR, rangeR, actor_list, ego_vehicle, radar_transform)
        

        # Iterate over all cars and store their values in RADAR_msgs
        for car in det_cars:
            radar_track = RadarTrack()
            # Assigning RADAR ID
            radar_track.track_id = car.id
            # Assingning three detected points
            radar_track.track_shape = polygon_list_to_ros_points(car.polygon)
            # Assigning linear velocity
            radar_track.linear_velocity = list_to_ros_vector3(car.velocity)
            # Append to main array
            msg.tracks.append(radar_track)

        # Header stamp and publish the message
        print('Sending')
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = '/map'
        pub.publish(msg)

        r.sleep()

def main():
    """
    main function for carla simulator ROS bridge
    maintaiing the communication client and the CarlaRosBridge objects
    """
    rospy.init_node("radar_client", anonymous=True)

    params = rospy.get_param('carla')
    host = params['host']
    port = params['port']

    rospy.loginfo("Trying to connect to {host}:{port}".format(host=host, port=port))

    try:
        carla_client = carla.Client(host=host, port=port)
        carla_client.set_timeout(2000)

        carla_world = carla_client.get_world()

        rospy.loginfo("Connected")

        # bridge_cls = CarlaRosBridge
        # carla_ros_bridge = bridge_cls(
        #     carla_world=carla_client.get_world(), params=params)
        # carla_ros_bridge.run()
        # carla_ros_bridge = None
        # Get all  vehicle actors in environment
        actor_list = carla_world.get_actors().filter('vehicle.*')
        npc_list = []

        # Get ego vehicle object and remove it from actor list
        ego_vehicle = None
        for actor in actor_list:
            attribute_val = actor.attributes['role_name']
            if attribute_val == 'hero':
                ego_vehicle = actor
            else:
                npc_list.append(actor)

        if ego_vehicle is not None:
            publisher(npc_list, ego_vehicle)

        rospy.logdebug("Delete world and client")
        del carla_world
        del carla_client

    finally:
        rospy.loginfo("Done")


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
