#!/usr/bin/env python

import rospy
from sensor_msgs.msg import PointCloud, PointCloud2
from geometry_msgs.msg import Pose, PoseStamped
from nav_msgs.msg import Odometry
import open3d as o3d
import numpy as np # numpy general
from open3d_ros_helper import open3d_ros_helper as orh # open3d ros helper, funciones para facilitarnos la vida
import copy
import timeit

class Loc:
    def __init__(self):
        rate = rospy.Rate(10)
        self.initial_pose = np.asarray([0.0,0.0,0.0,1.0]) # punto inicial ()
        self.icp_init = False # Bandera para ICP en instante inicial
        self.T_init=np.asarray([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]) # asumimos que no se mueve inicialmente
        self.punto=np.zeros((4,1))

        # topic con la pose para hallar la inicial
        pose_subs = rospy.Subscriber("/ual/pose", PoseStamped, self.subscriber_callback)

        # Para el GPS
        gps_sub = rospy.Subscriber("/mavros/global_position/local", Odometry, self.gps_callback)

        # cuando recibimos la posicion, continuamos
        while self.initial_pose[0] == 0.0:
            rate.sleep()

        # no necesitamos la pose inicial mas por lo que nos desuscribimos
        pose_subs.unregister()

        # publicador donde pondremos la posicion obtenida
        self.pos_T_pub = rospy.Publisher('/del_uav/ICP_position', Pose, queue_size=10)

        # nos suscribimos al topic del lidar
        print("[ICP] Waiting for Lidar topic...")
        laser_sub = rospy.Subscriber('/del_uav/uav_laser_scan', PointCloud2, self.sub_callback)
        print("[ICP] Lidar topic detected. Waiting for first PointCloud capture.")

        # tengo que hacer una captura inicial
        # La captura se hace en el callback
        while self.icp_init is False:
            rate.sleep()
        
        # guardo nube y punto como la de los instantes anteriores
        self.nube_ant = self.nube
        self.punto_ant = self.initial_pose
        print("[ICP] Fin del init")

    def subscriber_callback(self, data):
        self.initial_pose = np.asarray([data.pose.position.x, data.pose.position.y, data.pose.position.z, 1])
        self.orientation = np.asarray([data.pose.orientation.x, data.pose.orientation.y, data.pose.orientation.z])

        #self.punto[0] = round(self.punto[0],3)
        #self.punto[1] = round(self.punto[1],3)
        #self.punto[2] = round(self.punto[2],3)

        #self.orientation[0] = round(self.orientation[0],1)
        #self.orientation[1] = round(self.orientation[1],1)
        #self.orientation[2] = round(self.orientation[2],1)

    def icp_process(self):
        # Parto de la nube en el instante actual (self.nube) y la nube en el instante anterior (self.nube_ant)

        # estimacion de normal para PointToPlane
        #self.nube_ant.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))
        #self.nube.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))

        # --Inserta procedimientos de filtrado aqui--
        threshold = 0.1
        # ICP punto a punto (opcion por defecto) / Partimos de que el robot no se mueve de una posicion a la siguiente (opcion por defecto)
        start = timeit.default_timer()
        self.T = o3d.registration.registration_icp(self.nube_ant, self.nube, threshold, self.T_init, o3d.registration.TransformationEstimationPointToPoint())
        stop = timeit.default_timer()
        #self.T_init = self.T.transformation

        # esto imprime info de la convergencia
        print(self.T)
        print("Transformation is:")
        print(self.T.transformation) # la matriz de transformacion
        print("")
        #self.draw_registration_result()

        # calculo de punto, es T por el punto anterior, me da el nuevo
        self.punto = np.matmul(self.T.transformation, self.punto_ant)

        print("Tiempo de calculo ICP: ", stop-start)


        #test = np.transpose(self.punto)
        #self.new_point = np.matmul(self.T.transformation, test)
        print(self.punto) # imprimo el punto que me sale
        #print(self.punto)
        #print(self.pose_real)

        # publicacion del punto
        tolist = self.punto.tolist()
        tolist.pop(3)

        PoseMsg = Pose()
        PoseMsg.position.x = tolist[0]
        PoseMsg.position.y = tolist[1]
        PoseMsg.position.z = tolist[2]

        self.pos_T_pub.publish(PoseMsg)

        # actualizacion de variables
        self.punto_ant = self.punto
        self.nube_ant = self.nube

        # integracion de medidas
        #self.KalmanFilter()

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



    def gps_callback(self, data):
        self.gps_pose = Pose()
        self.gps_pose = data.pose.pose
        gps_cov_mat = np.array(data.pose.covariance)
        #self.gps_cov = np.delete(gps_cov_mat, np.s_([3:6]))
        


    def KalmanFilter(self):
        # definicion de variables
        # covarianza
        r = 0.1
        q_icp = 100
        #q_gps = # tengo ya la matriz de covarianza

        # construccion de matrices de covarianza
        Q_gps = np.diag(self.gps_cov)

        # Modelos del sistema
        A=np.array([ []  ])
        


if __name__ == '__main__':
    try:
        rospy.init_node('icp_node', anonymous=True)
        obj = Loc()

        #obj.icp_process()
        

        rospy.spin()
    except rospy.ROSInterruptException:
        pass