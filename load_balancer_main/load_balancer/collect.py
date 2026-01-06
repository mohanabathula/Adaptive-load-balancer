#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
import csv
import time
import subprocess
import os

class GPUFrameLogger:
    def __init__(self):
        rospy.init_node('gpu_frame_logger', anonymous=True)

        # === Parameters ===
        self.image_topic = rospy.get_param("~image_topic", "/camera/image_raw")
        self.csv_path = rospy.get_param("~csv_path", "/tmp/gpu_frame_log.csv")
        self.log_rate = rospy.get_param("~log_rate", 1.0)  # seconds

        self.frame_count = 0
        self.start_time = time.time()

        # Subscriber
        rospy.Subscriber(self.image_topic, Image, self.image_callback)

        # CSV setup
        self.csv_file = open(self.csv_path, 'w', newline='')
        self.writer = csv.writer(self.csv_file)
        self.writer.writerow([
            "timestamp",
            "elapsed_time_sec",
            "frame_count",
            "gpu_util_percent",
            "gpu_mem_used_MB"
        ])

        rospy.loginfo(f"Logging GPU + frames to {self.csv_path}")

        self.timer = rospy.Timer(
            rospy.Duration(self.log_rate),
            self.log_stats
        )

    def image_callback(self, msg):
        self.frame_count += 1

    def get_gpu_stats(self):
        """
        Uses nvidia-smi (works on Jetson & desktop NVIDIA GPUs)
        """
        try:
            output = subprocess.check_output(
                [
                    "nvidia-smi",
                    "--query-gpu=utilization.gpu,memory.used",
                    "--format=csv,noheader,nounits"
                ],
                stderr=subprocess.DEVNULL
            ).decode("utf-8").strip()

            gpu_util, mem_used = output.split(',')
            return int(gpu_util), int(mem_used)

        except Exception:
            return -1, -1  # GPU info unavailable

    def log_stats(self, event):
        timestamp = time.time()
        elapsed = timestamp - self.start_time
        gpu_util, mem_used = self.get_gpu_stats()

        self.writer.writerow([
            timestamp,
            round(elapsed, 2),
            self.frame_count,
            gpu_util,
            mem_used
        ])

        self.csv_file.flush()

    def shutdown(self):
        self.csv_file.close()


if __name__ == "__main__":
    logger = GPUFrameLogger()
    rospy.on_shutdown(logger.shutdown)
    rospy.spin()
