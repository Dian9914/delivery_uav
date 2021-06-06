#!/usr/bin/env python

from delivery_uav.srv import goto_srv
from geometry_msgs.msg import PoseStamped, TwistStamped
from uav_abstraction_layer.msg import State
import rospy
import time

class controller_server():
    def __init__(self):
        self.pose = [0, 0, 0]
        self.x_ref = self.pose[0]
        self.y_ref = self.pose[1]
        self.z_ref = self.pose[2]
        self.pub_vel = rospy.Publisher('ual/set_velocity', TwistStamped , queue_size=10)
        self.act_vel = TwistStamped()

        # inicializamos un error inicial de 0
        self.x_err_ant = 0;
        self.y_err_ant = 0;
        self.z_err_ant = 0;

        # inicializo el error acummulado a 0
        self.x_accumErr = 0;
        self.y_accumErr = 0;
        self.z_accumErr = 0;

        self.controller_state=False #bandera que marca si el control debe o no actuar


    def pos_control(self, dt):
        # CALCULO EL ERROR
        x_err = self.x_ref - self.pose[0]
        y_err = self.y_ref - self.pose[1]
        z_err = self.z_ref - self.pose[2]

        # APLICO EL ALGORITMO DE CONTROL
        # PRIMERA APROXIMACION: CONTROL P
        '''k_x = 0.25
        k_y = 0.25
        k_z = 0.35
        x_vel = k_x*x_err
        y_vel = k_y*y_err
        z_vel = k_z*z_err'''

        # SEGUNDA APROXIMACIoN, CONTROL PID
        # Parametros de control, hay que ajustar:
        Kp_x= 0.7
        Kp_y= 0.7
        Kp_z= 0.7

        Ki_x= 0.1
        Ki_y= 0.1
        Ki_z= 0.05

        Kd_x= 0.1
        Kd_y= 0.1
        Kd_z= 0.05

        windup_max=3.0
        windup_min=-3.0

        sat_max=2.0
        sat_min=-2.0

        # Calculo el error acummulado
        self.x_accumErr += x_err*dt;
        self.y_accumErr += y_err*dt;
        self.z_accumErr += z_err*dt;
        # aplicamos un factor de windup, que es basicamente una saturacion al error acumulado
        if self.x_accumErr>windup_max:
            self.x_accumErr=windup_max
        elif self.x_accumErr<windup_min:
            self.x_accumErr=windup_min

        if self.y_accumErr>windup_max:
            self.y_accumErr=windup_max
        elif self.y_accumErr<windup_min:
            self.y_accumErr=windup_min

        if self.z_accumErr>windup_max:
            self.z_accumErr=windup_max
        elif self.z_accumErr<windup_min:
            self.z_accumErr=windup_min
            

        # computo el control
        x_vel = Kp_x*x_err + Ki_x*self.x_accumErr + Kd_x*(x_err- self.x_err_ant)/dt;
        y_vel = Kp_y*y_err + Ki_y*self.y_accumErr + Kd_y*(y_err- self.y_err_ant)/dt;
        z_vel = Kp_z*z_err + Ki_z*self.z_accumErr + Kd_z*(z_err- self.z_err_ant)/dt;

        # saturo la senial
        if x_vel>sat_max:
            x_vel=sat_max
        elif x_vel<sat_min:
            x_vel=sat_min

        if y_vel>sat_max:
            y_vel=sat_max
        elif y_vel<sat_min:
            y_vel=sat_min

        if z_vel>sat_max:
            z_vel=sat_max
        elif z_vel<sat_min:
            z_vel=sat_min

        # guardo el error anterior
        self.x_err_ant = x_err;
        self.y_err_ant = y_err;
        self.z_err_ant = z_err;

        # PUBLICO LA ACCION DE CONTROL
        
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
        # Calculamos la diferencia de tiempo entre obtenciones de datos
        past_time = self.now_time
        self.now_time = time.time()
        dt=self.now_time-past_time
        # Guardamos los datos
        self.pose = [data.pose.position.x, data.pose.position.y, data.pose.position.z]
        self.pose[0] = round(self.pose[0],3)
        self.pose[1] = round(self.pose[1],3)
        self.pose[2] = round(self.pose[2],3)
        # llamamos al control solo si el controlador esta activado
        if self.controller_state:
            self.pos_control(dt)

    def start_subscriber(self):
        # inicializacion de los topic subscribers. En principio solo nos suscribimos al topic de posicion
        self.now_time = time.time()
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