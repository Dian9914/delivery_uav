#!/usr/bin/env python

from delivery_uav.srv import goto_srv
from geometry_msgs.msg import PoseStamped, TwistStamped
from uav_abstraction_layer.msg import State
import rospy

class controller_server():
    def __init__(self):
        self.pose = [0, 0, 0]
        self.x_ref = self.pose[0]
        self.y_ref = self.pose[1]
        self.z_ref = self.pose[2]
        self.pub_vel = rospy.Publisher('ual/set_velocity', TwistStamped , queue_size=10)
        self.act_vel = TwistStamped()

        self.controller_state=False #bandera que marca si el control debe o no actuar

    def pos_control(self):
        # CALCULO EL ERROR
        x_err = self.x_ref - self.pose[0]
        y_err = self.y_ref - self.pose[1]
        z_err = self.z_ref - self.pose[2]

        # APLICO EL ALGORITMO DE CONTROL
        # PRIMERA APROXIMACION: CONTROL P
        k_x = 0.25
        k_y = 0.25
        k_z = 0.35
        x_vel = k_x*x_err
        y_vel = k_y*y_err
        z_vel = k_z*z_err

        # PUBLICO LA ACCION DE CONTROL
        # SOLO SI EL CONTROL ESTA ACTIVADO
        if self.controller_state:
            self.act_vel.twist.linear.x=x_vel
            self.act_vel.twist.linear.y=y_vel
            self.act_vel.twist.linear.z=z_vel
            self.pub_vel.publish(self.act_vel)

        return True

    def ual_state_checker(self,data):
        # al leer el estado, comprobamos que el estado sea flying
        if data.state==4 and self.controller_state is False:
            # si el estado es flying, cambiamos la referencia a la pose actual (para mantenernos estables)
            self.x_ref = self.pose[0]
            self.y_ref = self.pose[1]
            self.z_ref = self.pose[2]
            # activamos la bandera para permitir que se publique la senial de control
            self.controller_state=True
            print("CONTROLLER NODE: System ready, position control started.")
        elif data.state!=4 and self.controller_state is True:
            self.controller_state=False
            print("CONTROLLER NODE: Position control waiting for system restart.")


    def subscriber_callback(self, data):
        self.pose = [data.pose.position.x, data.pose.position.y, data.pose.position.z]
        self.pose[0] = round(self.pose[0],3)
        self.pose[1] = round(self.pose[1],3)
        self.pose[2] = round(self.pose[2],3)
        self.pos_control()

    def start_subscriber(self):
        # inicializacion de los topic subscribers. En principio solo nos suscribimos al topic de posicion
        try:
            # IMPORTANTE:
            # CAMBIAR POR EL TOPIC CORRESPONDIENTE CUANDO EL LOCALIZADOR ESTE IMPLEMENTADO
            self.pose_subs=rospy.Subscriber("/ual/pose", PoseStamped, self.subscriber_callback)
        except:
            print('SUBSCRIBER HANDLER [CN]: Unexpected error subscribing to pose topic.')
            return False

        try:
            # iniciamos el subscriber que se preocupa de observar el estado de UAL
            self.ual_state_sub = rospy.Subscriber('/ual/state', State, self.ual_state_checker)
        except:
            print('SUBSCRIBER HANDLER [CN]: Unexpected error subscribing to ual/state.')
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
        print("CONTROLLER NODE: Position control waiting for system start.")
        rospy.spin()

if __name__ == "__main__":
    server = controller_server()
    server.start_server()