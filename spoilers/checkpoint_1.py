#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64
import numpy as np


class ScanListener:
    def __init__(self):
        # a subcriber that listens to 
        self.scan_sub = rospy.Subscriber('scan', LaserScan,
                self.scan_callback, queue_size=1)
        self.closest_pub = rospy.Publisher('closest_point', Float64, queue_size=10)
        self.farthest_pub = rospy.Publisher('farthest_point', Float64,
                                                            queue_size=10)

    def scan_callback(self, data):
        ranges = np.array(data.ranges)
        proc_ranges = ranges[np.logical_and(~np.isnan(ranges), ~np.isinf(ranges))]
        closest_dist = min(proc_ranges)
        farthest_dist = max(proc_ranges)
        self.closest_pub.publish(closest_dist)
        self.farthest_pub.publish(farthest_dist)

if __name__ == '__main__':
    rospy.init_node('scan_listener', anonymous=True)
    scan_listener = ScanListener()
    rospy.spin()

