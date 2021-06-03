#!/usr/bin/env python

import rospy
from sensor_msgs.msg import PointCloud, PointCloud2
from geometry_msgs.msg import Pose, PoseStamped, Twist
from nav_msgs.msg import Odometry
from mavros_msgs.msg import Altitude
from delivery_uav.srv import goto_srv
import open3d as o3d
import numpy as np # numpy general
from open3d_ros_helper import open3d_ros_helper as orh # open3d ros helper, funciones para facilitarnos la vida
import copy
import timeit

class Loc_KF:
    def __init__(self):
        # Get initial position
        if rospy.has_param('~sim_origin'):
            sim_origin = rospy.get_param('~sim_origin')
            #sim_origin.append(1.0)
        else:
            sim_origin = np.asarray([0.0, 0.0, 0.0])

        # Flags for initial measurements
        self.odom_ok = False
        self.gps_ok = False
        self.alt_ok = False

        # Topics we need to subscribe to
        self.odom_sub = rospy.Subscriber("/ual/odom", Odometry, self.odom_callback)

        self.gps_pose = Pose()
        self.gps_twist = Twist()
        self.gps_sub = rospy.Subscriber("/mavros/global_position/local", Odometry, self.gps_callback)

        self.alt_value = 0.0
        self.alt_sub = rospy.Subscriber("/mavros/altitude", Altitude, self.alt_callback)


        # Topics we need to publish to



        # Wait for initial measurements
        local_rate = rospy.Rate(30)
        while (self.odom_ok is False and self.gps_ok is False and self.alt_ok is False):
            local_rate.sleep()

        self.initial_pose = sim_origin
        #self.odom_data_ant = np.asarray([self.initial_pose[0], self.initial_pose[1], self.initial_pose[2], 
        #                            0, 0, 0, 0])  #prov

        # KF variables
        # Vector de estados mu = X,Y,Z,vx,vy,vz, con z las del altimetro
        self.mu = sim_origin
        self.mu = self.mu.append([0.0,0.0,0.0])    
        self.mu_p = self.mu 

        # sigma, Q_gps -> given by odom & sensors
        self.sigma = np.zeros(6,6)
        self.sigma_p = self.sigma

        # Q for altimeter
        q_alt = 5

        # Q for measurements
        self.Q = np.diag([self.odom_cov_pos, self.odom_cov_vel])
        self.Q[3,3] = q_alt

        # Observation model
        self.C = np.eye(6)

        # publicador donde pondremos la posicion obtenida
        #self.pos_T_pub = rospy.Publisher('/del_uav/ICP_position', Pose, queue_size=10)

        print("[KF] Fin del init")

    def Z_upd(self):
        # measurements (X,Y,Z,Xant,Yant,Zant)
        self.Z = np.asarray([self.gps_pose.position.x, self.gps_pose.position.y, self.alt_value,
                            self.gps_twist.linear.x, self.gps_twist.linear.y, self.gps_twist.linear.z])

    def KF_pred(self):
        # La mu predicha es la de la odometria, que tiene deriva que vamos a corregir
        self.mu_p = self.odom_data
        self.sigma_p = np.diag([self.odom_cov_pos, self.odom_cov_vel])

    def KF_act(self):
        # Kt=sigma_p*C'*(C*sigma_p*C'+Q)^-1;
        #temp = np.matmul(self.C, np.matmul(self.sigma, np.transpose(self.C))) + self.Q
        temp = np.linalg.multi_dot(self.C, self.sigma_p, np.transpose(self.C)) + self.Q
        #self.Kt = np.matmul(np.matmul(self.sigma_p, np.transpose(self.C)), np.linalg.inv(temp))

        self.Kt = np.linalg.multi_dot(self.sigma_p, np.transpose(self.C), np.linalg.inv(temp))

        #mu = mu_p + Kt*([Z(i,:) Z(i-1,:)]'-C*mu_p);
        #sigma = (eye(4)-Kt*C)*sigma_p; % cambia el eye aqui tambien

        self.mu = self.mu_p + np.matmul(self.Kt, (self.Z - np.matmul(self.C,self.mu_p)))
        self.sigma = np.matmul(np.eye(6) - np.matmul(self.Kt, self.C), self.sigma_p)  

        print(self.mu)
        print("")     
        


    def odom_callback(self, data):
        self.odom_data = np.asarray([data.pose.pose.position.x, data.pose.pose.position.y, data.pose.pose.position.z, 
                                     data.twist.twist.linear.x, data.twist.twist.linear.y, data.twist.twist.linear.z])
        self.odom_cov_pos = np.asarray([data.pose.covariance[4,4], data.pose.covariance[5,5], data.pose.covariance[6,6]])
        self.odom_cov_vel = np.asarray([data.twist.covariance[4,4], data.twist.covariance[5,5], data.twist.covariance[6,6]])

    def gps_callback(self, data):
        self.gps_pose = data.pose.pose
        self.gps_twist = data.twist.twist
        gps_cov_mat = np.array(data.pose.covariance)
        #self.gps_cov = np.delete(gps_cov_mat, np.s_([3:6]))

    def alt_callback(self, data):
        self.alt_value = data.local
        
    def T_from_odom(self, odom):
        T_trans=np.asarray([[1.0,0.0,0.0,odom[0]-self.odom_data_ant[0]],
                            [0.0,1.0,0.0,odom[1]-self.odom_data_ant[1]],
                            [0.0,0.0,1.0,odom[2]-self.odom_data_ant[2]],
                            [0.0,0.0,0.0,1.0]])

        return T_trans

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
        


if __name__ == '__main__':
    try:
        rospy.init_node('KF_node', anonymous=True)


        obj = Loc_KF(sim_origin)

        rate = rospy.Rate(30) # 30 Hz es la frecuencia de UAL

        while not rospy.is_shutdown():
            obj.KF_pred()
            obj.Z_upd()
            obj.KF_act()
            rate.sleep()

        #obj.ICP_location()
        

        rospy.spin()
    except rospy.ROSInterruptException:
        pass