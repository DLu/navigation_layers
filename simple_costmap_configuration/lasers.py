#!/usr/bin/python

import rospy
import sys
from sensor_msgs.msg import LaserScan

rospy.init_node('laserz')
pub = rospy.Publisher('/base_scan', LaserScan)
scan = LaserScan()
scan.header.frame_id = '/base_laser_link'
scan.angle_min = -2.26892805099
scan.angle_max = 2.26456475258
scan.angle_increment = 0.00436332309619
scan.time_increment = 1.73611115315e-05
scan.scan_time = 0.0500000007451
scan.range_min = 0.0230000000447
scan.range_max = 60.0

rate = rospy.Rate(20)
while not rospy.is_shutdown():
    rate.sleep()
    scan.header.stamp = rospy.Time.now()
    if '-x' in sys.argv:
        scan.ranges = [1.0]*200
        if int(scan.header.stamp.secs) % 10 < 5:
            scan.ranges += [float('inf')] * 640
        else:
            scan.ranges += [1.5] * 640
        scan.ranges += [1.0]*200
    else:
        scan.ranges= [float('inf')] * 1040
        
    pub.publish(scan)

