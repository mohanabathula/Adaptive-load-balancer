#!/usr/bin/env python3
import rospy
import math
import random
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
from carla_msgs.msg import CarlaEgoVehicleControl
from pynput import keyboard
import carla

# ================= Configuration Variables =================
TARGET_SPEED_KMH = 15.0
SAFE_DISTANCE = 2.0
FRONT_VEHICLE_TOPIC = '/carla/detection'
# ===========================================================

ego_odom = None
closest_distance = None
control_pub = None
ego_moving = False  # Flag: True when moving

def odom_callback(msg):
    global ego_odom
    ego_odom = msg

def distance_callback(msg):
    global closest_distance
    closest_distance = msg.data

def on_press(key):
    global ego_moving
    try:
        if key.char == 's':  # start moving
            ego_moving = True
            rospy.loginfo("Ego vehicle started")
        elif key.char == 'a':  # stop
            ego_moving = False
            rospy.loginfo("Ego vehicle stopped")
    except AttributeError:
        pass

def quaternion_to_euler(x, y, z, w):
    roll = math.atan2(2*(w*x + y*z), 1 - 2*(x*x + y*y))
    pitch = math.asin(2*(w*y - z*x))
    yaw = math.atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))
    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)

def get_ego_transform_from_odom(odom_msg):
    pos = odom_msg.pose.pose.position
    ori = odom_msg.pose.pose.orientation
    roll, pitch, yaw = quaternion_to_euler(ori.x, ori.y, ori.z, ori.w)
    return carla.Transform(
        location=carla.Location(x=pos.x, y=-pos.y, z=pos.z),
        rotation=carla.Rotation(roll=roll, pitch=pitch, yaw=yaw)
    )

def spawn_vehicles(client, world, ego_transform):
    yaw_rad = math.radians(ego_transform.rotation.yaw)

    dist_front = 30.0
    loc1 = carla.Location(
        x=ego_transform.location.x + dist_front * math.cos(yaw_rad),
        y=ego_transform.location.y + dist_front * math.sin(yaw_rad),
        z=ego_transform.location.z + 0.5
    )
    loc2 = carla.Location(
        x=ego_transform.location.x + dist_front * math.cos(yaw_rad),
        y=ego_transform.location.y + dist_front * math.sin(yaw_rad) + 3.0,
        z=ego_transform.location.z + 0.5
    )

    blueprint_library = world.get_blueprint_library()
    car_blueprints = [bp for bp in blueprint_library.filter('vehicle.*') if int(bp.get_attribute('number_of_wheels')) == 4]

    bp1 = random.choice(car_blueprints)
    bp2 = random.choice(car_blueprints)

    v1 = world.try_spawn_actor(bp1, carla.Transform(loc1, ego_transform.rotation))
    if v1:
        rospy.loginfo(f"Spawned static vehicle in front: {v1.type_id}")
        v1.set_autopilot(False)
        v1.set_simulate_physics(False)
    else:
        rospy.logwarn("Failed to spawn vehicle in front!")

    v2 = world.try_spawn_actor(bp2, carla.Transform(loc2, ego_transform.rotation))
    if v2:
        rospy.loginfo(f"Spawned static vehicle beside: {v2.type_id}")
        v2.set_autopilot(False)
        v2.set_simulate_physics(False)
    else:
        rospy.logwarn("Failed to spawn vehicle beside!")

    spectator = world.get_spectator()
    spectator.set_transform(carla.Transform(
        ego_transform.location + carla.Location(z=15),
        carla.Rotation(pitch=-90)
    ))

def control_ego():
    global closest_distance, control_pub, ego_moving

    target_speed = TARGET_SPEED_KMH / 3.6  # m/s
    control_msg = CarlaEgoVehicleControl()
    control_msg.steer = 0.0
    control_msg.hand_brake = False

    if not ego_moving or (closest_distance is not None and closest_distance <= SAFE_DISTANCE):
        # Stop ego vehicle
        control_msg.throttle = 0.0
        control_msg.brake = 1.0
    else:
        # Move ego vehicle
        control_msg.throttle = 0.4  # tune empirically for 15 km/h
        control_msg.brake = 0.0

    control_pub.publish(control_msg)

def main():
    global ego_odom, control_pub

    rospy.init_node('ego_vehicle_keyboard_control', anonymous=True)
    rospy.Subscriber('/carla/ego_vehicle/odometry', Odometry, odom_callback)
    rospy.Subscriber(FRONT_VEHICLE_TOPIC, Float32, distance_callback)

    control_pub = rospy.Publisher('/carla/ego_vehicle/vehicle_control_cmd', CarlaEgoVehicleControl, queue_size=1)

    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)
    world = client.get_world()

    # Start keyboard listener
    listener = keyboard.Listener(on_press=on_press)
    listener.start()

    rate = rospy.Rate(20)
    spawned = False

    while not rospy.is_shutdown():
        if ego_odom is not None and not spawned:
            ego_transform = get_ego_transform_from_odom(ego_odom)
            rospy.loginfo(f"Ego position: {ego_transform.location}")
            spawn_vehicles(client, world, ego_transform)
            spawned = True

        if ego_odom is not None:
            control_ego()

        rate.sleep()

if __name__ == "__main__":
    main()

