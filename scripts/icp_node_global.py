#!/usr/bin/env python

import rospy
from sensor_msgs.msg import PointCloud, PointCloud2
from geometry_msgs.msg import Pose, PoseStamped
from nav_msgs.msg import Odometry
from delivery_uav.srv import goto_srv
import open3d as o3d
import numpy as np # numpy general
from open3d_ros_helper import open3d_ros_helper as orh # open3d ros helper, funciones para facilitarnos la vida
import copy
import timeit

class Loc:
    def __init__(self, sim_origin):
        #self.rate = rospy.Rate(5) # deseamos funcionar a 10Hz siempre que sea posible
        self.icp_init = False # Bandera para ICP en instante inicial
        self.T_init=np.asarray([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]) # asumimos que no se mueve inicialmente
        self.punto=np.asarray([0.0,0.0,0.0,1.0])

        # inicializo la posicion segun el argumento dado
        self.initial_pose = sim_origin

        # Para el GPS
        #self.gps_sub = rospy.Subscriber("/mavros/global_position/local", Odometry, self.gps_callback)

        # Para la odometria
        self.odom_data_ant = np.asarray([self.initial_pose[0], self.initial_pose[1], self.initial_pose[2], 
                                    0, 0, 0, 0])  #prov

        #self.odom_sub = rospy.Subscriber("/ual/odom",Odometry, self.odom_callback)

        # publicador donde pondremos la posicion obtenida
        #self.pos_T_pub = rospy.Publisher('/del_uav/ICP_position', Pose, queue_size=10)

        # nos suscribimos al topic del lidar
        print("[ICP] Waiting for Lidar topic...")
        #laser_sub = rospy.Subscriber('/del_uav/uav_laser_scan', PointCloud2, self.sub_callback)
        print("[ICP] Lidar topic detected. Waiting for first PointCloud capture.")

        # creamos servicio para visualizar ICP
        #s = rospy.Service('/del_uav/visualize_ICP', goto_srv, self.draw_registration_result)

        print("[ICP] Fin del init")

    def icp_process(self, icp_nube, T_icp):
        # Parto de la nube en el instante actual (self.nube) y la nube en el instante anterior (self.nube_ant)

        # estimacion de normal para PointToPlane
        #self.nube_ant.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))
        #icp_nube.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))

        # --Inserta procedimientos de filtrado aqui--
        threshold = 0.05
        # ICP punto a punto (opcion por defecto) / Partimos de que el robot no se mueve de una posicion a la siguiente (opcion por defecto)
        start = timeit.default_timer()

        print("Initial alignment")
        #evaluation = o3d.registration.evaluate_registration(
        #    self.nube_ant, icp_nube, threshold, T_icp)
        #print(evaluation)

        #self.T = o3d.registration.registration_icp(self.nube_ant, icp_nube, threshold, T_icp, o3d.registration.TransformationEstimationPointToPoint())
        self.T = orh.p2p_icp_registration(self.nube_ant, icp_nube, n_points=4000)
        stop = timeit.default_timer()
        #self.T_init = self.T.transformation

        # esto imprime info de la convergencia
        print(self.T)
        print("Transformation is:")
        print(self.T[0].transformation) # la matriz de transformacion
        print("")
        #self.draw_registration_result(nube_icp)

        # calculo de punto, es T por el punto anterior, me da el nuevo
        self.punto = np.matmul(self.T[0].transformation, self.punto_ant)

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

    def sub_callback(self, data):
        self.nube = orh.rospc_to_o3dpc(data)
        # print(self.nube) # que hace este print?
        # print(np.asarray(self.nube.points)) # y que hace este print?
        # o3d.visualization.draw_geometries([self.nube]) # visualizar pointcloud
        # o3d.io.write_point_cloud("test_point_cloud.pcd", self.nube)

        self.icp_init = True

    def odom_callback(self, data):
        self.odom_data = np.asarray([data.pose.pose.position.x, data.pose.pose.position.y, data.pose.pose.position.z, 
                                    data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w])

    def gps_callback(self, data):
        self.gps_pose = Pose()
        self.gps_pose = data.pose.pose
        gps_cov_mat = np.array(data.pose.covariance)
        #self.gps_cov = np.delete(gps_cov_mat, np.s_([3:6]))
        
    def T_from_odom(self, odom):
        T_trans=np.asarray([[1.0,0.0,0.0,odom[0]-self.odom_data_ant[0]],
                            [0.0,1.0,0.0,odom[1]-self.odom_data_ant[1]],
                            [0.0,0.0,1.0,odom[2]-self.odom_data_ant[2]],
                            [0.0,0.0,0.0,1.0]])

        return T_trans
        


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

    def ICP_location(self):
        # tengo que hacer una captura inicial
        # La captura se hace en el callback
        # Espero a tener lectura
        while self.icp_init is False:
            self.rate.sleep()
        
        # guardo nube y punto como la de los instantes anteriores
        self.nube_ant = self.nube
        self.punto_ant = self.initial_pose

        while not rospy.is_shutdown():
            odom = self.odom_data
            T_icp = self.T_from_odom(odom)
            self.icp_process(self.nube, T_icp)
            self.odom_data_ant=self.odom_data
            self.rate.sleep()




def draw_registration_result(source, target, transformation):
    source_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)
    source_temp.paint_uniform_color([1, 0.706, 0])
    target_temp.paint_uniform_color([0, 0.651, 0.929])
    source_temp.transform(transformation)
    o3d.visualization.draw_geometries([source_temp, target_temp])


def preprocess_point_cloud(pcd, voxel_size):
    print(":: Downsample with a voxel size %.3f." % voxel_size)
    pcd_down = pcd.voxel_down_sample(voxel_size)

    radius_normal = voxel_size * 2
    print(":: Estimate normal with search radius %.3f." % radius_normal)
    pcd_down.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=30))

    radius_feature = voxel_size * 5
    print(":: Compute FPFH feature with search radius %.3f." % radius_feature)
    pcd_fpfh = o3d.registration.compute_fpfh_feature(
        pcd_down,
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_feature, max_nn=100))
    return pcd_down, pcd_fpfh

def prepare_dataset(voxel_size):
    print(":: Load two point clouds and disturb initial pose.")
    source = o3d.io.read_point_cloud("test_point_cloud.pcd")
    target = o3d.io.read_point_cloud("new_test.pcd")
    trans_init = np.asarray([[1.0, 0.0, 0.0, 2.0], [0.0, 1.0, 0.0, 0.0],
                            [0.0, 0.0, 1.0, -2.0], [0.0, 0.0, 0.0, 1.0]])
    source.transform(trans_init)
    draw_registration_result(source, target, np.identity(4))

    source_down, source_fpfh = preprocess_point_cloud(source, voxel_size)
    target_down, target_fpfh = preprocess_point_cloud(target, voxel_size)
    return source, target, source_down, target_down, source_fpfh, target_fpfh


def execute_global_registration(source_down, target_down, source_fpfh,
                            target_fpfh, voxel_size):
    distance_threshold = voxel_size * 1.5
    print(":: RANSAC registration on downsampled point clouds.")
    print("   Since the downsampling voxel size is %.3f," % voxel_size)
    print("   we use a liberal distance threshold %.3f." % distance_threshold)
    result = o3d.registration.registration_ransac_based_on_feature_matching(
        source_down, target_down, source_fpfh, target_fpfh, distance_threshold,
        o3d.registration.TransformationEstimationPointToPoint(False), 4, [
            o3d.registration.CorrespondenceCheckerBasedOnEdgeLength(0.9),
            o3d.registration.CorrespondenceCheckerBasedOnDistance(
                distance_threshold)
        ], o3d.registration.RANSACConvergenceCriteria(4000000, 500))
    return result


def refine_registration(source, target, source_fpfh, target_fpfh, voxel_size, result_ransac):
    distance_threshold = voxel_size * 0.4
    print(":: Point-to-plane ICP registration is applied on original point")
    print("   clouds to refine the alignment. This time we use a strict")
    print("   distance threshold %.3f." % distance_threshold)
    result = o3d.registration.registration_icp(
        source, target, distance_threshold, result_ransac.transformation,
        o3d.registration.TransformationEstimationPointToPoint())
    return result



def global_start():
    voxel_size = 0.05  # means 5cm for the dataset
    source, target, source_down, target_down, source_fpfh, target_fpfh = prepare_dataset(voxel_size)

    result_ransac = execute_global_registration(source_down, target_down,
                                                source_fpfh, target_fpfh,
                                                voxel_size)
    print(result_ransac.transformation)
    draw_registration_result(source_down, target_down,
                            result_ransac.transformation)

    result_icp = refine_registration(source, target, source_fpfh, target_fpfh,
                                    voxel_size, result_ransac)
    print(result_icp)
    draw_registration_result(source, target, result_icp.transformation)        
        


if __name__ == '__main__':
    try:
        global_start()
        exit()


        rospy.init_node('icp_node', anonymous=True)
        if rospy.has_param('~sim_origin'):
            sim_origin = rospy.get_param('~sim_origin')
            sim_origin.append(1.0)
        else:
            sim_origin = np.asarray([0.0, 0.0, 0.0, 1.0])

        obj = Loc(sim_origin)

        #obj.ICP_location()
        

        rospy.spin()
    except rospy.ROSInterruptException:
        pass