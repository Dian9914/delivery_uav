#!/usr/bin/env python

from delivery_uav.srv import goto_srv
from geometry_msgs.msg import PoseStamped, TwistStamped
import rospy

class controller_server():
    def __init__(self):
        self.pose = [0, 0, 0]
        self.x_ref = self.pose[0]
        self.y_ref = self.pose[1]
        self.z_ref = self.pose[2]
        self.pub_vel = rospy.Publisher('ual/set_velocity', TwistStamped , queue_size=10)
        self.act_vel = TwistStamped()

    def pos_control(self):
        # CALCULO EL ERROR
        x_err = self.x_ref - self.pose[0]
        y_err = self.y_ref - self.pose[1]
        z_err = self.z_ref - self.pose[2]

        # APLICO EL ALGORITMO DE CONTROL
        # PRIMERA APROXIMACION: CONTROL P
        k_x = 0.25
        k_y = 0.25
        k_z = 0.2
        x_vel = k_x*x_err
        y_vel = k_y*y_err
        z_vel = k_z*z_err

        # PUBLICO LA ACCION DE CONTROL
        self.act_vel.twist.linear.x=x_vel
        self.act_vel.twist.linear.y=y_vel
        self.act_vel.twist.linear.z=z_vel
        self.pub_vel.publish(self.act_vel)

        return True


    def subscriber_callback(self, data):
        self.pose = [data.pose.position.x, data.pose.position.y, data.pose.position.z]
        self.pose[0] = round(self.pose[0],1)
        self.pose[1] = round(self.pose[1],1)
        self.pose[2] = round(self.pose[2],1)
        self.pos_control()

    def start_subscriber(self):
        # inicializacion de los topic subscribers. En principio solo nos suscribimos al topic de posicion
        try:
            # IMPORTANTE:
            # CAMBIAR POR EL TOPIC CORRESPONDIENTE CUANDO EL LOCALIZADOR ESTE IMPLEMENTADO
            pose_subs=rospy.Subscriber("/ual/pose", PoseStamped, self.subscriber_callback)
        except:
            print('SUBSCRIBER HANDLER [CN]: Unexpected error.')
            return False

        return True

    def goto_handler(self, req):
        if req.waypoint.pose.position.x<=50 and req.waypoint.pose.position.y<=50 and req.waypoint.pose.position.z>=0:
            self.x_ref=req.waypoint.pose.position.x
            self.y_ref=req.waypoint.pose.position.y
            self.z_ref=req.waypoint.pose.position.z
            print("CONTROLLER NODE: Going to new coordinates: [%d, %d, %d]."%(self.x_ref,self.y_ref,self.z_ref))
        else:
            print("CONTROLLER NODE: Incorrect coordinates")
            return False
        return True

    def start_server(self):
        rospy.init_node('controller_node')
        self.start_subscriber()
        self.x_ref = self.pose[0]
        self.y_ref = self.pose[1]
        self.z_ref = self.pose[2]
        
        s = rospy.Service('/del_uav/goto', goto_srv, self.goto_handler)
        print("CONTROLLER NODE: Position control ready.")
        rospy.spin()

if __name__ == "__main__":
    server = controller_server()
    server.start_server()