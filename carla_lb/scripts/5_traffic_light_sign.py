#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import torch
import time
from yolov5_trt import YoLov5TRT

# Initialize ROS node
rospy.init_node('traffic_sign_detector', anonymous=True)

bridge = CvBridge()

device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
print("Device:", device)

classes = ['go', 'right', 'park', 'red', 'green', 'crosswalk']
model = YoLov5TRT('./traffic_signs_a40n.engine', './libmyplugins.so', classes)

RESIZED_WIDTH = 1200
RESIZED_HEIGHT = 800

def ros_image_callback(msg):
    """Callback that receives an Image message and performs inference."""
    try:
        # Convert ROS Image → OpenCV image
        img = bridge.imgmsg_to_cv2(msg, "bgr8")
    except Exception as e:
        rospy.logerr(f"cv_bridge error: {e}")
        return

    # Resize
    img_resized = cv2.resize(img, (RESIZED_WIDTH, RESIZED_HEIGHT))

    # Inference
    start = time.time()
    boxes, scores, class_ids = model.infer(img_resized)
    end = time.time()

    # Format outputs
    scores = [float(s) for s in scores]
    boxes = [[float(round(x, 3)) for x in b.tolist()] for b in boxes]

    rospy.loginfo(f"Inference results:")
    rospy.loginfo(f" - Classes   : {class_ids}")
    rospy.loginfo(f" - Scores    : {scores}")
    rospy.loginfo(f" - Boxes     : {boxes}")
    rospy.loginfo(f" - Time      : {end - start:.4f}s")

def main():
    rospy.Subscriber("/carla/ego_vehicle/rgb_front/imag", Image, ros_image_callback, queue_size=1)

    rospy.loginfo("Traffic sign detector node started. Listening to /camera/image_raw ...")
    rospy.spin()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    finally:
        model.destroy()

