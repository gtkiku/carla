#!/usr/bin/env python

# Copyright (c) 2021 Tampere University
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

import glob
import os
import sys

try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla

import random
import time

vehicle = None
vehicle_red = None
vehicle_stopped = False

def camera_callback(pxl_event):
    global vehicle
    global vehicle_stopped

    print("Red pixel count is: ", pxl_event.pixel_count)

    if vehicle_stopped:
        return

    if (pxl_event.pixel_count > 22000):
        print("pixel limit reached, stopping both vehicles")
        vehicle_stopped = True

        vehicle.set_autopilot(False)
#        vehicle.enable_constant_velocity(carla.Vector3D(0, 0, 0))
#        vehicle.disable_constant_velocity()
        vehicle.set_target_velocity(carla.Vector3D(0, 0, 0))
        ctrl = carla.VehicleControl(0.0, 0.0, 1.0)
        vehicle.apply_control(ctrl)

        vehicle_red.set_autopilot(False)
#        vehicle_red.enable_constant_velocity(carla.Vector3D(0, 0, 0))
#        vehicle_red.disable_constant_velocity()
        vehicle_red.set_target_velocity(carla.Vector3D(0, 0, 0))
        ctrl = carla.VehicleControl(0.0, 0.0, 1.0)
        vehicle_red.apply_control(ctrl)

def main():
    actor_list = []
    npc_list = []
    global vehicle
    global vehicle_red
    global vehicle_stopped

    # In this tutorial script, we are going to add a vehicle to the simulation
    # and let it drive in autopilot. We will also create a camera attached to
    # that vehicle, and use OpenCL to process camera frames

    try:
        # First of all, we need to create the client that will send the requests
        # to the simulator. Here we'll assume the simulator is accepting
        # requests in the localhost at port 2000.
        client = carla.Client('localhost', 2000)
        client.set_timeout(2.0)

        # Once we have a client we can retrieve the world that is currently
        # running.
        world = client.get_world()

        # The world contains the list blueprints that we can use for adding new
        # actors into the simulation.
        blueprint_library = world.get_blueprint_library()

        # for blue vehicle, choose dodge charger
        bp = blueprint_library.find("vehicle.dodge.charger_2020")
        # set it blue
        bp.set_attribute('color', '0,0,240')

        # for red vehicle choose ambulance
        bp_red = blueprint_library.find("vehicle.ford.ambulance")
        # set red color
        bp_red.set_attribute('color', '240,0,0')

        # list of recommended spawn points of the map.
        spawn_points = world.get_map().get_spawn_points()

        # spawn the vehicles at precise points
        transform = spawn_points[120] #
        transform_red = spawn_points[36] #
        vehicle = world.spawn_actor(bp, transform)
        vehicle_red = world.spawn_actor(bp_red, transform_red)

        # It is important to note that the actors we create won't be destroyed
        # unless we call their "destroy" function. If we fail to call "destroy"
        # they will stay in the simulation even after we quit the Python script.
        # For that reason, we are storing all the actors we create so we can
        # destroy them afterwards.
        actor_list.append(vehicle)
        actor_list.append(vehicle_red)
        print('created CAM VEH %s' % vehicle.type_id)
        print('created RED VEH %s' % vehicle_red.type_id)

        # Let's add now an "OpenCL RGB" camera attached to the blue vehicle.
        camera_bp = blueprint_library.find('sensor.camera.rgb_ocl')
        camera_transform = carla.Transform(carla.Location(x=1.5, z=2.4))
        camera = world.spawn_actor(camera_bp, camera_transform, attach_to=vehicle)
        actor_list.append(camera)
        print('created %s' % camera.type_id)

        # Now we register the function that will be called each time the sensor
        # receives the red pixel count. If the pixel count is higher than a
        # threshold, we stop both vehicles in the callback.
        camera.listen(camera_callback)

        # .... set autopilot on both vehicles
        vehicle.set_autopilot(True)
        vehicle_red.set_autopilot(True)

        # .... alternatively, enable constant velocity
        #vehicle.enable_constant_velocity(carla.Vector3D(5, 0, 0))
        #vehicle_red.enable_constant_velocity(carla.Vector3D(5, 0, 0))

        # don't enable the camera callback until all actors are spawned
        print("running simulation")
        time.sleep(15)
        print("finished simulation")

    finally:

        print('destroying actors')
        camera.destroy()
        client.apply_batch([carla.command.DestroyActor(x) for x in actor_list])
        print('done.')


if __name__ == '__main__':

    main()
