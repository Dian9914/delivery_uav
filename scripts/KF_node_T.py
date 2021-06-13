#!/usr/bin/env python

# Messages
from geometry_msgs.msg import Pose, Twist, Point, Vector3, Transform # Messages for the measurements and for T
from nav_msgs.msg import Odometry # Message for ICP odometry
from mavros_msgs.msg import Altitude # Message for altimeter
from delivery_uav.msg import sigma_value # Own message to publish sigma values
from uav_abstraction_layer.msg import State # UAL state, to check for ICP failures during takeoff
from rtabmap_ros.msg import OdomInfo # Message that cointains T, as well as other info for ICP debugging

# Services
from rtabmap_ros.srv import ResetPose # Service for resetting the ICP odometry in case of failure

# Python libraries
import rospy # Main library to work with ROS
import numpy as np # Useful for working with arrays
from open3d_ros_helper import open3d_ros_helper as orh # open3d ros helper, has useful functions for data transformations
from service_client import service_client # Our own library to work easily with ROS services

class Loc_KF:
    def __init__(self):
        # Get initial position from rosparam "sim_origin"
        if rospy.has_param('~sim_origin'):
            sim_origin = rospy.get_param('~sim_origin')
            #sim_origin.append(1.0)
        else:
            sim_origin = np.asarray([0.0, 0.0, 0.0])

        sim_origin = np.asarray([0.0, 0.0, 0.0]) # override previous conf ---PENDING TO CHECK LAUNCH FILES----

        # SUBSCRIBED TOPICS AND ASSOCIATED VARIABLES
        # UAL state + ICP reset for the failproof check
        self.ual_sub = rospy.Subscriber("/ual/state", State, self.ual_callback)
        self.reset_icp = service_client("/reset_odom_to_pose", ResetPose)
        self.reset_trigger = False # Used to flag if an ICP reset has happened

        # Visual odometry using ICP
        self.odom_sub = rospy.Subscriber("/odom", Odometry, self.odom_callback)
        # Data is directly transformed to a numpy array

        # GPS measurements
        self.gps_sub = rospy.Subscriber("/mavros/global_position/local", Odometry, self.gps_callback)
        self.gps_pose = Pose() # Object where the measured position goes to
        
        # Altimeter measurements
        self.alt_sub = rospy.Subscriber("/mavros/altitude", Altitude, self.alt_callback)
        self.alt_value = 0.0 # Variable where we save the measurement
        
        # Topic needed to find ICP transformation matrix
        self.T_sub = rospy.Subscriber("/odom_info", OdomInfo, self.T_callback)
        self.T = np.diag(np.ones(4)) # Initial value for T

        print("[KF] Sub topics OK")



        # PUBLISH TOPICS
        self.mu_pos_pub = rospy.Publisher("/del_uav/KF_pose", Point, queue_size=50) # KF position result
        self.mu_vel_pub = rospy.Publisher("/del_uav/KF_twist", Vector3, queue_size=50) # KF twist result
        self.sigma_pub = rospy.Publisher("/del_uav/KF_sigma", sigma_value, queue_size=50) # KF sigma for covariance

        # Objects needed for publishers
        self.mu_pos_pub_obj = Point()
        self.mu_vel_pub_obj = Vector3()
        self.sigma_obj = sigma_value()



        # INITIAL VALUES FOR KF OPERATION
        # Mu will consist of the position and speed values of the UAV. They are known values at t=0
        self.mu = sim_origin  
        self.mu_p = self.mu 

        # initial odometry value, in case a measurement has not been received yet
        self.odom_data = self.mu
        self.odom_cov = np.ones(3)
        

        # sigma, Q_gps -> given by odom & sensors
        # sigma is just initialized and its value comes directly from ICP odometry
        self.sigma = np.zeros([3,3]) # since we know the initial position
        self.sigma_p = np.zeros([3,3])

        # Defined covariance parameters for KF
        q_alt = 1E-6 # q_alt needs to be better than the model and GPS one, nearly doesn't take into account ICP Z prediction (bad)
        q_gps = 3E-4 # q_gps is quite good, 1E-4 for working KF, more for better prediction of real position
        r = 1E-5 # Covariance for the model

        # construct Q matrix from q covariances (GPS doesn't provide us with valid values)
        self.Q = np.diag(q_gps*np.ones(3)) # our GPS covariance

        # The altimeter has a different one
        self.Q[2,2] = q_alt

        # Observation model
        self.C = np.eye(3)

        # Static model and covariance for our system (since we'll be calculating sigma_p)
        self.A = np.eye(3)
        self.R = np.diag(r*np.ones(3))

        print("[KF] Init end")



    def Z_upd(self):
        # Measurements will consist of measured position by GPS and altimeter
        self.Z = np.asarray([self.gps_pose.position.x, self.gps_pose.position.y, self.alt_value])

    def KF_pred(self):
        # (T version of KF node) Based on the transformation matrix provided by rtabmap_ros, we can estimate
        # the point using T*point(k-1)
        self.mu_p = np.matmul(self.T, np.append(self.mu[0:3],1))
        self.mu_p = np.delete(self.mu_p, 3)

        # Sigma will be obtained with a model of our system assuming it's static
        self.sigma_p = np.linalg.multi_dot([self.A, self.sigma, np.transpose(self.A)]) + self.R # our covariance


    def KF_act(self):
        # KF calculations for update step, we will update using GPS measurements
        temp = np.linalg.multi_dot([self.C, self.sigma_p, np.transpose(self.C)]) + self.Q
        self.Kt = np.linalg.multi_dot([self.sigma_p, np.transpose(self.C), np.linalg.inv(temp)])
        self.mu = self.mu_p + np.matmul(self.Kt, (self.Z - np.matmul(self.C,self.mu_p)))
        self.sigma = np.matmul(np.eye(3) - np.matmul(self.Kt, self.C), self.sigma_p)  

        # Prints for debugging
        #print("Kt filtro")
        #print(self.Kt)
        #print("") 

        #print("Mu estimada")
        #print(self.mu)
        #print("")     

        #print("Datos GPS")
        #print(self.Z)
        #print("")

        #print("Sigma estimada")
        #print(self.sigma)
        #print("")

        # T result
        #print("T estimada")
        #print(self.T)
        #print("")


        # Publish the results to topics to be used externally
        # Position result
        self.mu_pos_pub_obj.x = self.mu[0]
        self.mu_pos_pub_obj.y = self.mu[1]
        self.mu_pos_pub_obj.z = self.mu[2]

        # Position covariance result
        self.sigma_obj.pos_cov.x = self.sigma[0,0]
        self.sigma_obj.pos_cov.y = self.sigma[1,1]
        self.sigma_obj.pos_cov.z = self.sigma[2,2]

        # Publish
        self.mu_pos_pub.publish(self.mu_pos_pub_obj)
        self.sigma_pub.publish(self.sigma_obj)
        


    def odom_callback(self, data):
        '''Extract data from an nav_msgs/Odometry message'''
        # Instead of defining an object, we'll build the odometry data directly as a numpy array
        self.odom_data = np.asarray([data.pose.pose.position.x, data.pose.pose.position.y, data.pose.pose.position.z])
        # Other way: np.reshape

        # Since we know the positions of the wanted elements on the covariance array, we'll just extract them
        self.odom_cov = np.asarray([data.pose.covariance[21], data.pose.covariance[28], data.pose.covariance[35]])

        # Debugging prints
        #print(self.odom_cov_pos)
        #print(self.odom_cov_vel)

    def gps_callback(self, data):
        '''Extract data from GPS'''
        self.gps_pose = data.pose.pose
        self.gps_cov_pos = np.asarray([data.pose.covariance[21], data.pose.covariance[28], data.pose.covariance[35]])
        #print(self.gps_cov_pos)
        #print(self.gps_cov_vel)

    def alt_callback(self, data):
        '''Extract data from altimeter'''
        self.alt_value = data.local
        
    def T_callback(self, data):
        '''Get transformation matrix T when we receive an OdomInfo msg'''
        self.T = orh.msg_to_se3(data.transform)
        

    def ual_callback(self, data):
        ''' Callback for the ICP failproof module'''
        if data.state == 3: # Corresponds to UAV taking off
            threshold = 0.7
            # Check ICP measurement is too far from GPS measurements
            # Since we're taking off, both need to be close to 0
            icp_err = abs(self.odom_data[0] - self.Z[0])
            if (icp_err > threshold):
                print("[KF] ------------ ICP, error found ---------------")
                print("[KF] Resetting ICP measurement")
                # Generate message object and publish to the service that resets ICP
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
                #print("ICP working OK")
                pass

            #print(self.reset_trigger)


if __name__ == '__main__':
    try:
        rospy.init_node('KF_node', anonymous=True)
        obj = Loc_KF()

        rate = rospy.Rate(30) # 30 Hz is the frecuency of MAVROS

        while not rospy.is_shutdown():
            obj.KF_pred()
            obj.Z_upd()
            obj.KF_act()
            
            rate.sleep()

        

        rospy.spin()
    except rospy.ROSInterruptException:
        pass