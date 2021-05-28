#!/usr/bin/env python

import rospy
from sensor_msgs.msg import PointCloud, PointCloud2
import open3d as o3d
import numpy as np # numpy general
from open3d_ros_helper import open3d_ros_helper as orh # open3d ros helper, funciones para facilitarnos la vida
import copy

class Loc:
    def __init__(self):
        self.icp_init = False # Bandera para ICP en instante inicial
        self.T_init=np.asarray([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]) # asumimos que no se mueve inicialmene

        rate = rospy.Rate(10)
        print("[ICP] Waiting for Lidar topic...")
        laser_sub = rospy.Subscriber('/del_uav/uav_laser_scan', PointCloud2, self.sub_callback)
        print("[ICP] Lidar topic detected. Waiting for first PointCloud capture.")
        # La captura se hace en el callback
        while self.icp_init is False:
            rate.sleep()
        self.nube_ant = self.nube
        #self.nube_ant = o3d.io.read_point_cloud("test_point_cloud_old.pcd")
        print("[ICP] Fin del init")

                

    def icp_process(self):
        # Parto de la nube en el instante actual (self.nube) y la nube en el instante anterior (self.nube_ant)
        print(self.nube)
        print(self.nube_ant)

        # --Inserta procedimientos de filtrado aqui--
        threshold = 0.02
        # ICP punto a punto (opcion por defecto) / Partimos de que el robot no se mueve de una posicion a la siguiente (opcion por defecto)
        self.T = o3d.registration.registration_icp(self.nube_ant, self.nube, threshold)

        print(self.T)
        print("Transformation is:")
        print(self.T.transformation)
        print("")
        #self.draw_registration_result()

    def draw_registration_result(self):
        source_temp = copy.deepcopy(self.nube_ant)
        target_temp = copy.deepcopy(self.nube)
        source_temp.paint_uniform_color([1, 0.706, 0])
        target_temp.paint_uniform_color([0, 0.651, 0.929])
        source_temp.transform(self.T.transformation)
        o3d.visualization.draw_geometries([source_temp, target_temp])

    def sub_callback(self, data):
        self.nube = orh.rospc_to_o3dpc(data)
        # print(self.nube) # que hace este print?
        # print(np.asarray(self.nube.points)) # y que hace este print?
        # o3d.visualization.draw_geometries([self.nube]) # visualizar pointcloud
        # o3d.io.write_point_cloud("test_point_cloud.pcd", self.nube)

        if (self.icp_init == True):
            self.icp_process()

        self.icp_init = True
        


if __name__ == '__main__':
    try:
        rospy.init_node('icp_node', anonymous=True)
        obj = Loc()

        #obj.icp_process()
        

        rospy.spin()
    except rospy.ROSInterruptException:
        pass