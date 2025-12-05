#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
import psutil
import subprocess
import json
import time

# Application info
APPLICATION_INFO = {
    "application": "object_detection",
    "latency_sensitivity": "high",
    "accuracy_priority": "high"
}
# Host for RTT measurement
PING_HOST = "192.168.20.16"

# ROS topic name
TOPIC_NAME = "/carla_res_util"

def get_system_usage():
    """Return CPU, RAM, GPU usage percentages."""
    # CPU usage (%)
    cpu_percent = psutil.cpu_percent(interval=1)

    # RAM usage (%)
    ram_percent = psutil.virtual_memory().percent

    # GPU usage using nvidia-smi
    gpu_percent = 0
    try:
        result = subprocess.check_output(
            ["nvidia-smi", "--query-gpu=utilization.gpu", "--format=csv,noheader,nounits"],
            universal_newlines=True
        )
        gpu_percent = int(result.strip().split("\n")[0])
    except Exception as e:
        rospy.logwarn(f"GPU usage read error: {e}")

    return {"cpu_percent": cpu_percent, "ram_percent": ram_percent, "gpu_percent": gpu_percent}

def get_network_bandwidth(interval=1):
    """Return network upload/download speed in Mbps over the given interval."""
    net1 = psutil.net_io_counters()
    time.sleep(interval)
    net2 = psutil.net_io_counters()

    sent_speed = (net2.bytes_sent - net1.bytes_sent) * 8 / (interval * 1024 * 1024)  # Mbps
    recv_speed = (net2.bytes_recv - net1.bytes_recv) * 8 / (interval * 1024 * 1024)  # Mbps

    return {"upload_Mbps": round(sent_speed, 2), "download_Mbps": round(recv_speed, 2)}

def get_rtt(host=PING_HOST):
    """Return RTT to a host in milliseconds."""
    try:
        output = subprocess.check_output(
            ["ping", "-c", "1", host],
            stderr=subprocess.STDOUT,
            universal_newlines=True
        )
        for line in output.split("\n"):
            if "time=" in line:
                return round(float(line.split("time=")[1].split(" ")[0]), 2)
    except Exception as e:
        rospy.logwarn(f"RTT measurement error: {e}")
    return None

def main():
    rospy.init_node('carla_res_util', anonymous=True)
    pub = rospy.Publisher(TOPIC_NAME, String, queue_size=1)
    rate = rospy.Rate(10)  # Publish every 2 seconds

    rospy.loginfo("CARLA Resource Monitor node started")

    while not rospy.is_shutdown():
        usage = get_system_usage()
        network = get_network_bandwidth(interval=1)
        rtt = get_rtt(PING_HOST)

        message = {
            "application": APPLICATION_INFO["application"],
            "latency_sensitivity": APPLICATION_INFO["latency_sensitivity"],
            "accuracy_priority": APPLICATION_INFO["accuracy_priority"],
            "cpu_percent": usage["cpu_percent"],
            "ram_percent": usage["ram_percent"],
            "gpu_percent": usage["gpu_percent"],
            "network_upload_Mbps": network["upload_Mbps"],
            "network_download_Mbps": network["download_Mbps"],
            "rtt_ms": rtt
        }

        pub.publish(String(data=json.dumps(message)))
        rospy.loginfo(f"Published resource usage: {message}")

        rate.sleep()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass

