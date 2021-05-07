#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan


def sub_callback(data):
    print(min(data.ranges))

        
if __name__ == '__main__':
    try:
        rospy.init_node('find_min_lidar', anonymous=True)

        pose_subscriber = rospy.Subscriber('/uav_laser_scan',
                                                LaserScan, sub_callback)
        rate = rospy.Rate(10)

        rospy.spin()
    except rospy.ROSInterruptException:
        pass