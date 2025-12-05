#!/usr/bin/env python3
import carla
import random
import time

# Connect to CARLA server
client = carla.Client('localhost', 2000)
client.set_timeout(10.0)
world = client.get_world()
blueprint_library = world.get_blueprint_library()
spawn_points = world.get_map().get_spawn_points()

# Vehicle and pedestrian blueprints
VEHICLES = blueprint_library.filter('vehicle.*')
PEDESTRIANS = blueprint_library.filter('walker.pedestrian.*')
NUM_VEHICLES = 10
NUM_PEDESTRIANS = 15

# Spawn vehicles with autopilot
vehicles = []
random.shuffle(spawn_points)
for i in range(min(NUM_VEHICLES, len(spawn_points))):
    bp = random.choice(VEHICLES)
    transform = spawn_points[i]
    vehicle = world.try_spawn_actor(bp, transform)
    if vehicle:
        vehicle.set_autopilot(True)  # Enable autopilot
        vehicles.append(vehicle)
        print(f"Spawned vehicle {vehicle.id} at {transform.location}")

# Spawn pedestrians
pedestrians = []
walker_controller_bp = blueprint_library.find('controller.ai.walker')

for i in range(NUM_PEDESTRIANS):
    spawn_point = carla.Transform()
    spawn_point.location.x = random.uniform(-100, 100)
    spawn_point.location.y = random.uniform(-100, 100)
    spawn_point.location.z = 0.5

    ped_bp = random.choice(PEDESTRIANS)
    pedestrian = world.try_spawn_actor(ped_bp, spawn_point)
    if pedestrian:
        # Spawn controller attached to the pedestrian
        controller = world.try_spawn_actor(walker_controller_bp, spawn_point, attach_to=pedestrian)
        if controller:
            controller.start()
            controller.go_to_location(carla.Location(
                x=random.uniform(-100, 100),
                y=random.uniform(-100, 100),
                z=0.0))
            controller.set_max_speed(1.5 + random.random())
            pedestrians.append((pedestrian, controller))
            print(f"Spawned pedestrian {pedestrian.id} with controller {controller.id}")


print(f"Total vehicles: {len(vehicles)}, Total pedestrians: {len(pedestrians)}")

try:
    while True:
        time.sleep(1)  # Keep script alive
except KeyboardInterrupt:
    print("Destroying actors...")
    for vehicle in vehicles:
        vehicle.destroy()
    for ped, controller in pedestrians:
        controller.destroy()
        ped.destroy()
    print("All actors destroyed.")

