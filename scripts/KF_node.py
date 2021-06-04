#!/usr/bin/env python

import rospy
from sensor_msgs.msg import PointCloud, PointCloud2
from geometry_msgs.msg import Pose, PoseStamped, Twist, Point, Vector3
from nav_msgs.msg import Odometry
from mavros_msgs.msg import Altitude
from delivery_uav.srv import goto_srv
from delivery_uav.msg import sigma_value
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

        print("[KF] Sub topics OK")

        # Topics we need to publish to
        self.mu_pos_pub = rospy.Publisher("/del_uav/KF_pose", Point, queue_size=50)
        self.mu_vel_pub = rospy.Publisher("/del_uav/KF_twist", Vector3, queue_size=50)
        self.sigma_pub = rospy.Publisher("/del_uav/KF_sigma", sigma_value, queue_size=50)
        self.mu_pos_pub_obj = Point()
        self.mu_vel_pub_obj = Vector3()
        self.sigma_obj = sigma_value()

        # Wait for initial measurements
        local_rate = rospy.Rate(30)
        while (self.odom_ok is False and self.gps_ok is False and self.alt_ok is False):
            print("Estoy en bucle")
            local_rate.sleep()
            
        print("[KF] Measurements obtained")

        self.initial_pose = sim_origin
        #self.odom_data_ant = np.asarray([self.initial_pose[0], self.initial_pose[1], self.initial_pose[2], 
        #                            0, 0, 0, 0])  #prov

        # KF variables
        # Vector de estados mu = X,Y,Z,vx,vy,vz, con z las del altimetro
        self.mu = sim_origin
        np.append(self.mu, [0.0,0.0,0.0])    
        self.mu_p = self.mu 

        # sigma, Q_gps -> given by odom & sensors
        self.sigma = np.zeros([6,6])
        self.sigma_p = self.sigma

        # Q for altimeter
        q_alt = 5
        q_gps = 1
        r = 0.5

        # Q for measurements
        #self.Q = np.diag(np.append(self.gps_cov_pos, self.gps_cov_vel)) # obtained GPS covariance
        self.Q = np.diag(q_gps*np.ones(6)) # our GPS covariance
        self.Q[2,2] = q_alt

        # Observation model
        self.C = np.eye(6)


        # extra for model covariance
        self.A = np.eye(6)
        self.R = np.diag(r*np.ones(6))

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
        #self.sigma_p = np.diag(np.append(self.odom_cov_pos, self.odom_cov_vel))
        self.sigma_p = np.linalg.multi_dot([self.A, self.sigma, np.transpose(self.A)]) + self.R # our covariance

    def KF_act(self):
        # Kt=sigma_p*C'*(C*sigma_p*C'+Q)^-1;
        #temp = np.matmul(self.C, np.matmul(self.sigma, np.transpose(self.C))) + self.Q
        temp = np.linalg.multi_dot([self.C, self.sigma_p, np.transpose(self.C)]) + self.Q
        #self.Kt = np.matmul(np.matmul(self.sigma_p, np.transpose(self.C)), np.linalg.inv(temp))

        self.Kt = np.linalg.multi_dot([self.sigma_p, np.transpose(self.C), np.linalg.inv(temp)])

        #mu = mu_p + Kt*([Z(i,:) Z(i-1,:)]'-C*mu_p);
        #sigma = (eye(4)-Kt*C)*sigma_p; % cambia el eye aqui tambien

        self.mu = self.mu_p + np.matmul(self.Kt, (self.Z - np.matmul(self.C,self.mu_p)))
        self.sigma = np.matmul(np.eye(6) - np.matmul(self.Kt, self.C), self.sigma_p)  

        # Print for debugging
        print("Mu estimada")
        print(self.mu)
        print("")     

        print("Datos odometria")
        print(self.odom_data)

        print("Kt del filtro")
        print(self.Kt)

        # Result publishing
        self.mu_pos_pub_obj.x = self.mu[0]
        self.mu_pos_pub_obj.y = self.mu[1]
        self.mu_pos_pub_obj.z = self.mu[2]
        self.sigma_obj.pos_cov.x = self.sigma[0,0]
        self.sigma_obj.pos_cov.y = self.sigma[1,1]
        self.sigma_obj.pos_cov.z = self.sigma[2,2]

        self.mu_vel_pub_obj.x = self.mu[3]
        self.mu_vel_pub_obj.y = self.mu[4]
        self.mu_vel_pub_obj.z = self.mu[5]
        self.sigma_obj.vel_cov.x = self.sigma[3,3]
        self.sigma_obj.vel_cov.y = self.sigma[4,4]
        self.sigma_obj.vel_cov.z = self.sigma[5,5]

        self.mu_pos_pub.publish(self.mu_pos_pub_obj)
        self.mu_vel_pub.publish(self.mu_vel_pub_obj)
        self.sigma_pub.publish(self.sigma_obj)
        


    def odom_callback(self, data):
        self.odom_data = np.asarray([data.pose.pose.position.x, data.pose.pose.position.y, data.pose.pose.position.z, 
                                     data.twist.twist.linear.x, data.twist.twist.linear.y, data.twist.twist.linear.z])
        # Alternativa: Hacer reshape
        self.odom_cov_pos = np.asarray([data.pose.covariance[21], data.pose.covariance[28], data.pose.covariance[35]])
        self.odom_cov_vel = np.asarray([data.twist.covariance[21], data.twist.covariance[28], data.twist.covariance[35]])
        #print(self.odom_cov_pos)
        #print(self.odom_cov_vel)
        self.odom_ok = True

    def gps_callback(self, data):
        self.gps_pose = data.pose.pose
        self.gps_twist = data.twist.twist
        self.gps_cov_pos = np.asarray([data.pose.covariance[21], data.pose.covariance[28], data.pose.covariance[35]])
        self.gps_cov_vel = np.asarray([data.twist.covariance[21], data.twist.covariance[28], data.twist.covariance[35]])
        #print(self.gps_cov_pos)
        #print(self.gps_cov_vel)
        self.gps_ok = True

    def alt_callback(self, data):
        self.alt_value = data.local
        self.alt_ok = True
        
    def T_from_odom(self, odom):
        T_trans=np.asarray([[1.0,0.0,0.0,odom[0]-self.odom_data_ant[0]],
                            [0.0,1.0,0.0,odom[1]-self.odom_data_ant[1]],
                            [0.0,0.0,1.0,odom[2]-self.odom_data_ant[2]],
                            [0.0,0.0,0.0,1.0]])

        return T_trans
        


if __name__ == '__main__':
    try:
        rospy.init_node('KF_node', anonymous=True)


        obj = Loc_KF()

        rate = rospy.Rate(30) # 30 Hz es la frecuencia de UAL

        while not rospy.is_shutdown():
            obj.KF_pred()
            obj.Z_upd()
            obj.KF_act()
            rate.sleep()

        #obj.ICP_location()
        

        #rospy.spin()
    except rospy.ROSInterruptException:
        pass