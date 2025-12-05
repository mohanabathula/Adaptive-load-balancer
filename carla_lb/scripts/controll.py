#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from carla_msgs.msg import CarlaEgoVehicleControl
from pynput import keyboard
import math, json

# ================= Configuration =================
TARGET_SPEED_KMH = 20.0
SAFE_DISTANCE = 2.5
DETECTION_TOPIC = '/carla/edge_detection'
ODOM_TOPIC = '/carla/ego_vehicle/odometry'
CONTROL_TOPIC = '/carla/ego_vehicle/vehicle_control_cmd'
MAX_THROTTLE = 0.8
TARGET_LABEL = "car_1"   # ? Target vehicle to detect
# ==================================================

ego_moving = False
closest_distance = None
ego_odom = None
control_pub = None

# ================= Callbacks =================
def detection_callback(msg):
    """Extract distance of car_1 from detection string data"""
    global closest_distance
    try:
        # Convert string to list
        detections = json.loads(msg.data)

        found = False
        for obj in detections:
            if obj.get("unique_label") == TARGET_LABEL:
                closest_distance = float(obj.get("distance"))
                found = True
                break

        if not found:
            closest_distance = None  # car_1 not visible

    except Exception as e:
        rospy.logwarn(f"[Detection Parse Error] {e}")
        closest_distance = None

def odom_callback(msg):
    global ego_odom
    ego_odom = msg

def on_press(key):
    global ego_moving
    try:
        if key.char == 's':
            ego_moving = True
            print("\n? START command received ? Vehicle moving...")
        elif key.char == 'a':
            ego_moving = False
            print("\n? STOP command received ? Applying brakes!")
    except AttributeError:
        pass

# ================= Helper =================
def get_speed(odom_msg):
    vel = odom_msg.twist.twist.linear
    vx = vel.x
    vy = vel.y
    return math.sqrt(vx*vx + vy*vy) * 3.6

# ================= Control Loop =================
def control_ego():
    global ego_moving, closest_distance, ego_odom, MAX_THROTTLE

    control_msg = CarlaEgoVehicleControl()
    #control_msg.manual_gear_shift = True
    #control_msg.gear = 1
    control_msg.steer = 0.0
    control_msg.hand_brake = False
    control_msg.reverse = False

    current_speed = get_speed(ego_odom) if ego_odom else 0.0
    dist_str = f"{closest_distance:.2f} m" if closest_distance else "N/A"

    # --- MAIN DECISION LOGIC ---
    if not ego_moving:
        control_msg.throttle = 0.0
        control_msg.brake = 1.0
        print(f"? Stopped | Speed: {current_speed:.1f} km/h | Distance: {dist_str}")

    elif closest_distance and closest_distance <= SAFE_DISTANCE:
        control_msg.throttle = 0.0
        control_msg.brake = 1.0
        rospy.logwarn(f"?? EMERGENCY BRAKE! Too close ({dist_str}) | Speed: {current_speed:.1f} km/h")

    else:
    
        if current_speed < TARGET_SPEED_KMH:
            control_msg.throttle = 0.8
            control_msg.brake = 0.0
           
            print(f"? Moving | Speed: {current_speed:.1f} km/h | Distance: {dist_str}")
        else:
            control_msg.throttle = 0.0
            control_msg.brake = 0.0
            print(f"?? Cruising | Speed: {current_speed:.1f} km/h | Distance: {dist_str}")

    control_pub.publish(control_msg)

# ================= Main =================
def main():
    global control_pub

    rospy.init_node('ego_control_with_detection', anonymous=True)

    # Subscribers
    rospy.Subscriber(DETECTION_TOPIC, String, detection_callback)
    rospy.Subscriber(ODOM_TOPIC, Odometry, odom_callback)

    # Publisher
    control_pub = rospy.Publisher(CONTROL_TOPIC, CarlaEgoVehicleControl, queue_size=1)

    # Keyboard listener
    listener = keyboard.Listener(on_press=on_press)
    listener.start()

    print("\n=========== KEY CONTROLS ===========")
    print(" Press 's'  START vehicle")
    print(" Press 'a'  STOP vehicle (Brake)")
    print("====================================\n")

    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        control_ego()
        rate.sleep()

if __name__ == "__main__":
    main()

