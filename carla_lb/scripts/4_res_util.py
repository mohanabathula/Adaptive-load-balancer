#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
import psutil
from pynvml import nvmlInit, nvmlDeviceGetHandleByIndex, nvmlDeviceGetUtilizationRates

def get_cpu_gpu_usage():
    # Non-blocking CPU percent
    cpu_percent = psutil.cpu_percent(interval=None)
    # GPU usage
    handle = nvmlDeviceGetHandleByIndex(0)
    util = nvmlDeviceGetUtilizationRates(handle)
    return cpu_percent, util.gpu, util.memory

def main():
    rospy.init_node('res_util_monitor', anonymous=True)
    pub = rospy.Publisher('carla_lb/res_util', String, queue_size=10)

    nvmlInit()
    handle = nvmlDeviceGetHandleByIndex(0)

    rate = rospy.Rate(10)  # 1 Hz

    # Warm-up CPU measurement
    psutil.cpu_percent(interval=None)

    while not rospy.is_shutdown():
        # call twice for accurate CPU percent
        psutil.cpu_percent(interval=None)
        cpu, gpu, mem = get_cpu_gpu_usage()
        msg = f"CPU: {cpu:.1f}% | GPU: {gpu}% | GPU_MEM: {mem}%"
        print(msg, flush=True)
        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

