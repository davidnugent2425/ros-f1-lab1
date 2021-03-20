#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from lab1.msg import ScanRange
import numpy as np


class ScanListener:
    def __init__(self):
        self.scan_sub = rospy.Subscriber('scan', LaserScan,
                self.scan_callback, queue_size=1)
        self.scan_range_pub = rospy.Publisher('scan_range', ScanRange,
                queue_size=10)

    def scan_callback(self, data):
        ranges = np.array(data.ranges)
        proc_ranges = ranges[np.logical_and(~np.isnan(ranges), ~np.isinf(ranges))]
        closest_dist = min(proc_ranges)
        farthest_dist = max(proc_ranges)
        scan_range_msg = ScanRange(data.header, closest_dist, farthest_dist)
        self.scan_range_pub.publish(scan_range_msg)

if __name__ == '__main__':
    rospy.init_node('scan_listener', anonymous=True)
    scan_listener = ScanListener()
    rospy.spin()

