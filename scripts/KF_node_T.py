#!/usr/bin/env python

import rospy
from sensor_msgs.msg import PointCloud, PointCloud2
from geometry_msgs.msg import Pose, PoseStamped, Twist, Point, Vector3, Transform
from nav_msgs.msg import Odometry
from mavros_msgs.msg import Altitude
from delivery_uav.srv import goto_srv
from rtabmap_ros.srv import ResetPose
from delivery_uav.msg import sigma_value
from uav_abstraction_layer.msg import State
import open3d as o3d
import numpy as np # numpy general
from open3d_ros_helper import open3d_ros_helper as orh # open3d ros helper, funciones para facilitarnos la vida
import copy
import timeit

from rtabmap_ros.msg import OdomInfo

from service_client import service_client

class Loc_KF:
    def __init__(self):
        # Get initial position
        if rospy.has_param('~sim_origin'):
            sim_origin = rospy.get_param('~sim_origin')
            #sim_origin.append(1.0)
        else:
            sim_origin = np.asarray([0.0, 0.0, 0.0])

        sim_origin = np.asarray([0.0, 0.0, 0.0])
        ual_sub = rospy.Subscriber("/ual/state", State, self.ual_callback)
        self.KF_start = False
        self.reset_icp = service_client("/reset_odom_to_pose", ResetPose)
        self.reset_trigger = False

        # Flags for initial measurements
        self.odom_ok = False
        self.gps_ok = False
        self.alt_ok = False

        # Topics we need to subscribe to
        # Using visual odometry
        self.odom_sub = rospy.Subscriber("/odom", Odometry, self.odom_callback)

        self.gps_pose = Pose()
        self.gps_twist = Twist()
        self.gps_sub = rospy.Subscriber("/mavros/global_position/local", Odometry, self.gps_callback)

        self.alt_value = 0.0
        self.alt_sub = rospy.Subscriber("/mavros/altitude", Altitude, self.alt_callback)


        # Topic to find ICP transformation matrix
        self.T_sub = rospy.Subscriber("/odom_info", OdomInfo, self.T_callback)
        self.T = np.diag(np.ones(4))

        print("[KF] Sub topics OK")

        # Topics we need to publish to
        self.mu_pos_pub = rospy.Publisher("/del_uav/KF_pose", Point, queue_size=50)
        #self.mu_vel_pub = rospy.Publisher("/del_uav/KF_twist", Vector3, queue_size=50)
        self.sigma_pub = rospy.Publisher("/del_uav/KF_sigma", sigma_value, queue_size=50)
        self.mu_pos_pub_obj = Point()
        #self.mu_vel_pub_obj = Vector3()
        self.sigma_obj = sigma_value()

        # Wait for initial measurements
        local_rate = rospy.Rate(30)
        # ----- PENDIENTE: este bucle no funciona -----
        while (self.odom_ok != False and self.gps_ok != False and self.alt_ok != False):
            print("Estoy en bucle")
            local_rate.sleep()
            
        print("[KF] Measurements obtained")

        self.initial_pose = sim_origin
        #self.odom_data_ant = np.asarray([self.initial_pose[0], self.initial_pose[1], self.initial_pose[2], 
        #                            0, 0, 0, 0])  #prov

        # KF variables
        # Vector de estados mu = X,Y,Z,vx,vy,vz, con z las del altimetro
        self.mu = sim_origin
        #self.mu = np.append(self.mu, [0.0,0.0,0.0])    
        self.mu_p = self.mu 

        # initial odometry value ---PENDING TO REMOVE---
        self.odom_data = self.mu
        self.odom_cov = np.ones(3)
        

        # sigma, Q_gps -> given by odom & sensors
        # sigma is just initialized and its value comes directly from rtabmap_ros
        self.sigma = np.zeros([3,3])
        #self.sigma_p = self.sigma
        self.sigma_p = np.zeros([3,3])

        # Q for altimeter
        q_alt = 1E-6 # q_alt needs to be better than the model and GPS one, nearly doesn't take into account ICP Z prediction (bad)
        q_gps = 1E-3 # q_gps is quite good, 1E-4 for working KF, more for better prediction of real position
        r = 1E-5 # Covariance for the model
        #speed_cov_adj = 10 # estimated speed by GPS is not that good either

        # Q for measurements
        #self.Q = np.diag(np.append(self.gps_cov_pos, self.gps_cov_vel)) # obtained GPS covariance
        self.Q = np.diag(q_gps*np.ones(3)) # our GPS covariance

        # The altimeter has a different one
        self.Q[2,2] = q_alt

        # Speed covariance for GPS is higher
        #self.Q[3,3] = speed_cov_adj*self.Q[3,3]
        #self.Q[4,4] = speed_cov_adj*self.Q[4,4]
        #self.Q[5,5] = speed_cov_adj*self.Q[5,5]

        # Observation model
        self.C = np.eye(3)


        # extra for model covariance --NOT USED--
        self.A = np.eye(3)
        self.R = np.diag(r*np.ones(3))

        # publicador donde pondremos la posicion obtenida
        #self.pos_T_pub = rospy.Publisher('/del_uav/ICP_position', Pose, queue_size=10)

        print("[KF] Fin del init")

    def Z_upd(self):
        # measurements (X,Y,Z)
        self.Z = np.asarray([self.gps_pose.position.x, self.gps_pose.position.y, self.alt_value])

    def KF_pred(self):
        # La mu predicha es la de la odometria, que tiene deriva que vamos a corregir
        #self.mu_p = self.odom_data

        # Variante con T
        self.mu_p = np.matmul(self.T, np.append(self.mu[0:3],1))
        self.mu_p = np.delete(self.mu_p, 3)


        #self.sigma_p = np.diag(np.append(self.odom_cov_pos, self.odom_cov_vel))
        #self.sigma_p = np.linalg.multi_dot([self.A, self.sigma, np.transpose(self.A)]) + self.R # our covariance
        self.sigma_p = np.diag(self.odom_cov)

        # Variante con T
        self.sigma_p = np.linalg.multi_dot([self.A, self.sigma, np.transpose(self.A)]) + self.R # our covariance


    def KF_act(self):
        # Kt=sigma_p*C'*(C*sigma_p*C'+Q)^-1;
        #temp = np.matmul(self.C, np.matmul(self.sigma, np.transpose(self.C))) + self.Q
        temp = np.linalg.multi_dot([self.C, self.sigma_p, np.transpose(self.C)]) + self.Q
        #self.Kt = np.matmul(np.matmul(self.sigma_p, np.transpose(self.C)), np.linalg.inv(temp))

        self.Kt = np.linalg.multi_dot([self.sigma_p, np.transpose(self.C), np.linalg.inv(temp)])
        print("Kt filtro")
        print(self.Kt)
        print("") 


        #mu = mu_p + Kt*([Z(i,:) Z(i-1,:)]'-C*mu_p);
        #sigma = (eye(4)-Kt*C)*sigma_p; % cambia el eye aqui tambien

        self.mu = self.mu_p + np.matmul(self.Kt, (self.Z - np.matmul(self.C,self.mu_p)))
        self.sigma = np.matmul(np.eye(3) - np.matmul(self.Kt, self.C), self.sigma_p)  

        # Print for debugging
        print("Mu estimada")
        print(self.mu)
        print("")     

        print("Datos GPS")
        print(self.Z)
        print("")

        #print("Sigma estimada")
        #print(self.sigma)
        #print("")

        # T result
        #print("T estimada")
        #print(self.T)
        #print("")
        #print("Estimated point")

        # Result publishing
        self.mu_pos_pub_obj.x = self.mu[0]
        self.mu_pos_pub_obj.y = self.mu[1]
        self.mu_pos_pub_obj.z = self.mu[2]
        self.sigma_obj.pos_cov.x = self.sigma[0,0]
        self.sigma_obj.pos_cov.y = self.sigma[1,1]
        self.sigma_obj.pos_cov.z = self.sigma[2,2]

        #self.mu_vel_pub_obj.x = self.mu[3]
        #self.mu_vel_pub_obj.y = self.mu[4]
        #self.mu_vel_pub_obj.z = self.mu[5]
        #self.sigma_obj.vel_cov.x = self.sigma[3,3]
        #self.sigma_obj.vel_cov.y = self.sigma[4,4]
        #self.sigma_obj.vel_cov.z = self.sigma[5,5]

        self.mu_pos_pub.publish(self.mu_pos_pub_obj)
        #self.mu_vel_pub.publish(self.mu_vel_pub_obj)
        self.sigma_pub.publish(self.sigma_obj)
        


    def odom_callback(self, data):
        # Extract data from an nav_msgs/Odometry message
        self.odom_data = np.asarray([data.pose.pose.position.x, data.pose.pose.position.y, data.pose.pose.position.z])
        # Alternativa: Hacer reshape
        #self.odom_cov_pos = np.asarray([data.pose.covariance[21], data.pose.covariance[28], data.pose.covariance[35]])
        #self.odom_cov_vel = np.asarray([data.twist.covariance[21], data.twist.covariance[28], data.twist.covariance[35]])

        self.odom_cov = np.asarray([data.pose.covariance[21], data.pose.covariance[28], data.pose.covariance[35]])

        #print(self.odom_cov_pos)
        #print(self.odom_cov_vel)
        self.odom_ok = True

    def gps_callback(self, data):
        self.gps_pose = data.pose.pose
        self.gps_twist = data.twist.twist
        self.gps_cov_pos = np.asarray([data.pose.covariance[21], data.pose.covariance[28], data.pose.covariance[35]])
        #self.gps_cov_vel = np.asarray([data.twist.covariance[21], data.twist.covariance[28], data.twist.covariance[35]])
        #print(self.gps_cov_pos)
        #print(self.gps_cov_vel)
        self.gps_ok = True

    def alt_callback(self, data):
        self.alt_value = data.local
        self.alt_ok = True
        
    def T_callback(self, data):
        self.T = orh.msg_to_se3(data.transform)

    #def get_KF_start(self):
        #return self.KF_start
        

    def ual_callback(self, data):
        if data.state == 3: # si el UAV esta despegando
            threshold = 0.7
            # comprobacion de que el ICP da medida correcta (para la X)
            icp_err = abs(self.odom_data[0] - self.Z[0])
            if (icp_err > threshold):
                print(" ------------ ICP, error detectado ---------------")
                # tengo que generar el mensaje
                reset_icp_msg = ResetPose._request_class()
                reset_icp_msg.x = self.Z[0]
                reset_icp_msg.y = self.Z[1]
                reset_icp_msg.z = self.Z[2]
                reset_icp_msg.roll = 0.0
                reset_icp_msg.pitch = 0.0
                reset_icp_msg.yaw = 0.0
                self.reset_icp.single_response(reset_icp_msg)
                self.reset_trigger = True
            else:
                print("ICP funciona OK")

            print(self.reset_trigger)
            #self.KF_start = True
            #response = self.reset_icp.single_response(#mensaje)




if __name__ == '__main__':
    try:
        rospy.init_node('KF_node', anonymous=True)


        obj = Loc_KF()

        rate = rospy.Rate(30) # 30 Hz es la frecuencia de UAL

        while not rospy.is_shutdown():
            #if obj.get_KF_start() == True:
            obj.KF_pred()
            obj.Z_upd()
            obj.KF_act()
            
            rate.sleep()

        #obj.ICP_location()
        

        #rospy.spin()
    except rospy.ROSInterruptException:
        pass