#!/usr/bin/env python3
import rospy
import math
import random
import carla
from nav_msgs.msg import Odometry

# Global variable to store ego odometry
ego_odom = None

def odom_callback(msg):
    global ego_odom
    ego_odom = msg

def quaternion_to_euler(x, y, z, w):
    """Convert quaternion to roll, pitch, yaw in degrees."""
    # Roll
    roll = math.atan2(2*(w*x + y*z), 1 - 2*(x*x + y*y))
    # Pitch
    pitch = math.asin(2*(w*y - z*x))
    # Yaw
    yaw = math.atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))
    # Convert to degrees
    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)

def get_ego_transform_from_odom(odom_msg):
    pos = odom_msg.pose.pose.position
    ori = odom_msg.pose.pose.orientation
    roll, pitch, yaw = quaternion_to_euler(ori.x, ori.y, ori.z, ori.w)
    # Convert ROS ENU ? CARLA UE
    return carla.Transform(
        location=carla.Location(x=pos.x, y=-(pos.y), z=pos.z),
        rotation=carla.Rotation(roll=roll, pitch=pitch, yaw=yaw)
    )

def spawn_vehicles(client, world, ego_transform):
    yaw_rad = math.radians(ego_transform.rotation.yaw)

    # Vehicle 1: 10m in front
    dist_front = 30.0
    loc1 = carla.Location(
        x=ego_transform.location.x + dist_front * math.cos(yaw_rad),
        y=ego_transform.location.y + dist_front * math.sin(yaw_rad),
        z=ego_transform.location.z + 0.5  # z offset for visibility
    )

    # Vehicle 2: beside ego (right side), z + 0.5, y offset +3
    loc2 = carla.Location(
        x=ego_transform.location.x + dist_front * math.cos(yaw_rad),
        y=ego_transform.location.y + dist_front * math.sin(yaw_rad) + 3.0,
        z=ego_transform.location.z + 0.3
    )

    blueprint_library = world.get_blueprint_library()
    # Filter only cars
    car_blueprints = [bp for bp in blueprint_library.filter('vehicle.mini.*') if int(bp.get_attribute('number_of_wheels')) == 4]

    bp1 = random.choice(car_blueprints)
    bp2 = random.choice(car_blueprints)

    # Spawn first vehicle
    v1 = world.try_spawn_actor(bp1, carla.Transform(loc1, ego_transform.rotation))
    if v1:
        rospy.loginfo(f"Spawned static vehicle in front: {v1.type_id}")
        v1.set_autopilot(False)
        v1.set_simulate_physics(False)
    else:
        rospy.logwarn("Failed to spawn vehicle in front!")

    # Spawn second vehicle
    v2 = world.try_spawn_actor(bp2, carla.Transform(loc2, ego_transform.rotation))
    if v2:
        rospy.loginfo(f"Spawned static vehicle beside: {v2.type_id}")
        v2.set_autopilot(False)
        v2.set_simulate_physics(False)
    else:
        rospy.logwarn("Failed to spawn vehicle beside!")

    # Move spectator above to see them
    spectator = world.get_spectator()
    spectator.set_transform(carla.Transform(
        ego_transform.location + carla.Location(z=15),
        carla.Rotation(pitch=-90)
    ))

def main():
    global ego_odom
    rospy.init_node('spawn_static_cars_node', anonymous=True)
    rospy.Subscriber('/carla/ego_vehicle/odometry', Odometry, odom_callback)

    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)
    world = client.get_world()

    rate = rospy.Rate(2)  # 2 Hz
    spawned = False

    while not rospy.is_shutdown():
        if ego_odom is not None and not spawned:
            ego_transform = get_ego_transform_from_odom(ego_odom)
            rospy.loginfo(f"Ego position: {ego_transform.location}")
            rospy.loginfo(f"Ego yaw: {ego_transform.rotation.yaw}")
            spawn_vehicles(client, world, ego_transform)
            spawned = True  # spawn only once
        rate.sleep()

if __name__ == "__main__":
    main()

