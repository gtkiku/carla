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

vehicle = False
vehicle_stopped = False
all_spawned = False

def camera_callback(pxl_event):
    global vehicle
    global vehicle_stopped
    global all_spawned
    if vehicle_stopped:
        print("vehicle stopped")
        return
    if not all_spawned:
#        print("not all NPC spawned")
        return

    print("Red pixel count is: ", pxl_event.pixel_count)
    if (pxl_event.pixel_count > 10000):
        print("pixel limit reached, stopping vehicle")
        vehicle_stopped = True
        vehicle.set_autopilot(False)
        vehicle.set_target_velocity(carla.Vector3D(0, 0, 0))
        ctrl = carla.VehicleControl(0.0, 0.0, 1.0)
        vehicle.apply_control(ctrl)

def main():
    actor_list = []
    npc_list = []
    global vehicle
    global vehicle_stopped
    global all_spawned

    # In this tutorial script, we are going to add a vehicle to the simulation
    # and let it drive in autopilot. We will also create a camera attached to
    # that vehicle, and save all the images generated by the camera to disk.

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

        # Now let's filter all the blueprints of type 'vehicle' and choose one
        # at random.
        bp = random.choice(blueprint_library.filter('vehicle'))

        # A blueprint contains the list of attributes that define a vehicle's
        # instance, we can read them and modify some of them. For instance,
        # let's randomize its color.
        if bp.has_attribute('color'):
            color = random.choice(bp.get_attribute('color').recommended_values)
            bp.set_attribute('color', color)

        # Now we need to give an initial transform to the vehicle. We choose a
        # random transform from the list of recommended spawn points of the map.
        indx = 162
        transform = world.get_map().get_spawn_points()[indx]

        # So let's tell the world to spawn the vehicle.
        vehicle = world.spawn_actor(bp, transform)

        # It is important to note that the actors we create won't be destroyed
        # unless we call their "destroy" function. If we fail to call "destroy"
        # they will stay in the simulation even after we quit the Python script.
        # For that reason, we are storing all the actors we create so we can
        # destroy them afterwards.
        actor_list.append(vehicle)
        print('created %s' % vehicle.type_id)

        # Let's put the vehicle to drive around.

        # Let's add now a "depth" camera attached to the vehicle. Note that the
        # transform we give here is now relative to the vehicle.
        camera_bp = blueprint_library.find('sensor.camera.rgb_ocl')
        camera_transform = carla.Transform(carla.Location(x=1.5, z=2.4))
        camera = world.spawn_actor(camera_bp, camera_transform, attach_to=vehicle)
        actor_list.append(camera)
        print('created %s' % camera.type_id)

        # Now we register the function that will be called each time the sensor
        # receives the red pixel count. If the pixel count is higher than a
        # threshold, we stop the vehicle.
#        cc = carla.ColorConverter.LogarithmicDepth
        camera.listen(camera_callback)

        # Oh wait, I don't like the location we gave to the vehicle, I'm going
        # to move it a bit forward.
#        location = vehicle.get_location()
#        location.x += 40
#        vehicle.set_location(location)
#        print('moved vehicle to %s' % location)

        # But the city now is probably quite empty, let's add a few more
        # vehicles.
#        transform.location += carla.Location(x=40, y=-3.2)
#        transform.rotation.yaw = -180.0
        for _ in range(0, 40):
#            transform.location.x += 8.0

            transform = random.choice(world.get_map().get_spawn_points())
            bp = random.choice(blueprint_library.filter('vehicle'))

            # This time we are using try_spawn_actor. If the spot is already
            # occupied by another object, the function will return None.
            npc = world.try_spawn_actor(bp, transform)
            if npc is not None:
                actor_list.append(npc)
                npc_list.append(npc)
                print('created %s' % npc.type_id)

        # set autopilot on all
        vehicle.set_autopilot(True)
        for npc in npc_list:
            npc.set_autopilot(True)

        # don't enable the camera callback until all actors are spawned
        all_spawned = True
        print("running simulation")
        time.sleep(20)
        print("finished simulation")

    finally:

        print('destroying actors')
        camera.destroy()
        client.apply_batch([carla.command.DestroyActor(x) for x in actor_list])
        print('done.')


if __name__ == '__main__':

    main()
