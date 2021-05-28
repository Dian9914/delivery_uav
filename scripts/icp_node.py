#!/usr/bin/env python

import rospy
from sensor_msgs.msg import PointCloud, PointCloud2
import open3d as o3d
import numpy as np # numpy general
from open3d_ros_helper import open3d_ros_helper as orh # open3d ros helper, funciones para facilitarnos la vida

class Loc:
    def __init__(self):
        pass



    def sub_callback(self, data):
        pcd = orh.rospc_to_o3dpc(data)
        print(pcd)
        print(np.asarray(pcd.points))
        o3d.visualization.draw_geometries([pcd])


if __name__ == '__main__':
    try:
        rospy.init_node('icp_process', anonymous=True)
        obj = Loc()

        pose_subscriber = rospy.Subscriber('/del_uav/uav_laser_scan',
                                                PointCloud2, obj.sub_callback)
        rate = rospy.Rate(10)

        rospy.spin()
    except rospy.ROSInterruptException:
        pass