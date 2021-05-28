import rospy
from sensor_msgs.msg import PointCloud2
import open3d as o3d
import numpy as np # numpy general

class Loc:
    def __init__(self, data):
        pass



    def sub_callback(self, data):
        pass


if __name__ == '__main__':
    try:
        rospy.init_node('icp_process', anonymous=True)
        obj = Loc()

        pose_subscriber = rospy.Subscriber('/del_uav/uav_laser_scan',
                                                PointCloud2, Loc.sub_callback)
        rate = rospy.Rate(10)

        rospy.spin()
    except rospy.ROSInterruptException:
        pass