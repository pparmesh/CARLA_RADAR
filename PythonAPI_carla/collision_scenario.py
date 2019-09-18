#!/usr/bin/env python

# Copyright (c) 2017 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""Spawn Crashing vehicle into the simulation"""

import glob
import os
import sys

try:
    sys.path.append(glob.glob('**/*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla

import argparse
import random
import time
import math

from agents.navigation.controller import VehiclePIDController

class Colliding_Agent:
    def __init__(self, world, main_agent):
        self.world = world
        self.map = self.world.get_map()
        self.vehicle = None
        self.main_agent = main_agent
        self.target_speed = 30 #Km/Hr
        dt = 0.05
        self.args_lateral_dict = {
            'K_P': 1.95,
            'K_D': 0.01,
            'K_I': 1.4,
            'dt': dt}
        self.args_longitudinal_dict = {
            'K_P': 1.0,
            'K_D': 0,
            'K_I': 1,
            'dt': dt}
        self.vehicle_controller = None
        self.waypoint = None

    def create_agent(self, spawn_point):
        try:
            blueprints = self.world.get_blueprint_library().filter('vehicle.*')
            blueprints = [x for x in blueprints if int(x.get_attribute('number_of_wheels')) == 4]
            blueprints = [x for x in blueprints if not x.id.endswith('isetta')]

            # spawn_points = list(self.world.get_map().get_spawn_points())

            # print('found %d spawn points.' % len(spawn_points))

            blueprint = random.choice(blueprints)
            if blueprint.has_attribute('color'):
                color = random.choice(blueprint.get_attribute('color').recommended_values)
                blueprint.set_attribute('color', color)
            blueprint.set_attribute('role_name', 'autopilot')
            self.vehicle = self.world.try_spawn_actor(blueprint, spawn_point)
            if self.vehicle is not None:
                self.vehicle.set_autopilot()
                print('spawned %r at %s' % (self.vehicle.type_id, spawn_point.location))
            else:
                print("No vehicles to spawn")
                return False

            self.vehicle_controller = VehiclePIDController(self.vehicle, args_lateral=self.args_lateral_dict, \
                                                       args_longitudinal=self.args_longitudinal_dict)
        
        except:
            print("Unable to import vehicle")


    def destroy(self):
        if self.vehicle is not None:
            self.vehicle.destroy()  

    def control_agent(self):
        if self.find_distance(self.vehicle.get_transform(), self.main_agent.get_transform()) < 50:
            if self.waypoint is None:
                self.waypoint = self.map.get_waypoint(self.vehicle.get_transform().location, True)
            # print(self.main_agent.get_transform())
            # print(waypoint.next(50)[0].get_left_lane())
            # waypoint.lane_width
            self.vehicle.set_autopilot(False)
            target_waypoint = self.waypoint.next(25)[0].get_left_lane()
            # print(target_waypoint)
            control = self.vehicle_controller.run_step(self.target_speed, target_waypoint)
            self.vehicle.apply_control(control)

    def find_distance(self, main_agent_T, vehicle_T):
        distance = math.sqrt((main_agent_T.location.x - vehicle_T.location.x)**2 + (main_agent_T.location.y - vehicle_T.location.y)**2 \
        + (main_agent_T.location.z - vehicle_T.location.z)**2)
        return distance

class NPC:
    def __init__(self, world, N, spawn_points):
        self.world = world
        self.N = N
        self.blueprints = None
        self.actor_list = []
        self.spawn_points = spawn_points
        self.create_npc()

    def try_spawn_random_vehicle_at(self, transform):
        blueprint = random.choice(self.blueprints)
        if blueprint.has_attribute('color'):
            color = random.choice(blueprint.get_attribute('color').recommended_values)
            blueprint.set_attribute('color', color)
        blueprint.set_attribute('role_name', 'autopilot')
        print(blueprint)
        vehicle = self.world.try_spawn_actor(blueprint, transform)
        print("vehicle")
        if vehicle is not None:
            self.actor_list.append(vehicle)
            vehicle.set_autopilot()
            # print('*NPC* spawned %r at %s' % (vehicle.type_id, transform.location))
            return True
        return False

    def create_npc(self):
        #try:
            self.blueprints = self.world.get_blueprint_library().filter('vehicle.*')
            self.blueprints = [x for x in self.blueprints if int(x.get_attribute('number_of_wheels')) == 4]
            self.blueprints = [x for x in self.blueprints if not x.id.endswith('isetta')] #and not x.id.endswith('carlacola')]
            spawn_points = self.spawn_points
            random.shuffle(spawn_points)

            print('*NPC* found %d spawn points.' % len(spawn_points))

            count = self.N

            for spawn_point in spawn_points:
                if self.try_spawn_random_vehicle_at(spawn_point):
                    count -= 1
                if count <= 0:
                    break

            while count > 0:
                time.sleep(2)
                if self.try_spawn_random_vehicle_at(random.choice(spawn_points)):
                    count -= 1

            print('*NPC* spawned %d vehicles, press Ctrl+C to exit.' % self.N)
            time.sleep(5)

        # except:
        #     print("*NPC* Unable to import vehicle")


    def destroy(self, client):
        print('\n*NPC* destroying %d actors' % len(self.actor_list))
        client.apply_batch([carla.command.DestroyActor(x.id) for x in self.actor_list])


# def main():
#     argparser = argparse.ArgumentParser(
#         description=__doc__)
#     argparser.add_argument(
#         '--host',
#         metavar='H',
#         default='127.0.0.1',
#         help='IP of the host server (default: 127.0.0.1)')
#     argparser.add_argument(
#         '-p', '--port',
#         metavar='P',
#         default=2000,
#         type=int,
#         help='TCP port to listen to (default: 2000)')
#     args = argparser.parse_args()

#     actor_list = []
#     client = carla.Client(args.host, args.port)
#     client.set_timeout(2.0)

#     # ca = Collliding_Agent(client.get_world())
#     # ca.control_agent( client)

#     try:

#         world = client.get_world()
#         blueprints = world.get_blueprint_library().filter('vehicle.*')

#         def try_spawn_random_vehicle_at(transform):
#             blueprint = random.choice(blueprints)
#             if blueprint.has_attribute('color'):
#                 color = random.choice(blueprint.get_attribute('color').recommended_values)
#                 blueprint.set_attribute('color', color)
#             blueprint.set_attribute('role_name', 'autopilot')
#             vehicle = world.try_spawn_actor(blueprint, transform)
#             if vehicle is not None:
#                 actor_list.append(vehicle)
#                 vehicle.set_autopilot()
#                 print('spawned %r at %s' % (vehicle.type_id, transform.location))
#                 return True
#             return False

#         # @todo Needs to be converted to list to be shuffled.
#         spawn_points = list(world.get_map().get_spawn_points())
#         #random.shuffle(spawn_points)

#         print('found %d spawn points.' % len(spawn_points))

#         try_spawn_random_vehicle_at(spawn_points[0])

#         #print('spawned vehicle at %d, press Ctrl+C to exit.' %spawn_points[0] )
#         x = 0
#         while True:
#             time.sleep(2)
#             x += 1
#             if x >= 5:
#                 car = actor_list[0]
#                 car.set_autopilot(False)
#                 control = carla.VehicleControl(throttle = 1.0, steer = 0.2, brake = 0.0, \
#                     hand_brake = False, reverse = False, manual_gear_shift = False, gear = 0)
#                 car.apply_control(control)

#     finally:
#         actor_list[0].destroy()

if __name__ == '__main__':

    try:
        main()
    except KeyboardInterrupt:
        pass
    finally:
        print('\ndone.')

# todo 
# Improve spawning of vehicles 
# make spawn npc and this a single code