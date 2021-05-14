#!/usr/bin/env python

from delivery_uav.srv import gripper_srv, user_interface
import rospy
from threading import Lock, Condition

class user_interface_server():
    home = [0, 0, 0]
    goal = [0, 0, 0]

    mtx_auto = Lock()
    mtx_ready = Lock()
    mtx_started = Lock()
    mtx_charge = Lock()
    vc_started = Condition(mtx_started)
    vc_ready = Condition(mtx_ready)
    vc_charge = Condition(mtx_charge)
    
    ready = False       #Variable que representa si el UAV esta en condicion de recibir ordenes
    started = False     #Variable que representa si el sistema esta inicializado y el UAV despegado
    charge = True       #Variable que representa si la carga esta a bordo

    travel = False        #Variable que representa si el UAV ya esta dirigiendose a un punto
    mtx_travel = Lock()
    vc_travel = Condition(mtx_travel)


    def start_sys(self):
        #Inicio del sistema, despega el UAV, guarda la posicion inicial como HOME y permite el resto de llamadas
        print("CENTRAL NODE: Starting the delivery UAV system.")


    def auto_mode(self, goal):
        #MODO AUTOMATICO: Principal modo de funcionamiento, el UAV se desplaza a un punto dado
        print("CENTRAL NODE: Starting the auto mode.")

    def idle_mode(self):
        #MODO AUTOMATICO: Principal modo de funcionamiento, el UAV se desplaza a un punto dado
        print("CENTRAL NODE: Starting the idle mode.")

    def drop_charge(self):
        #MODO AUTOMATICO: Principal modo de funcionamiento, el UAV se desplaza a un punto dado
        print("CENTRAL NODE: Starting the drop of the charge.")


        self.mtx_charge.acquire()
        self.charge=False # LA CARGA YA NO ESTA A BORDO
        self.vc_charge.notify_all()
        self.mtx_charge.release()

    def gohome(self):
        #MODO AUTOMATICO: Principal modo de funcionamiento, el UAV se desplaza a un punto dado
        print("CENTRAL NODE: Starting the way to the home mode.")



        # El sistema ya no esta iniciado
        self.mtx_started.acquire()
        self.started = False
        self.mtx_started.release()

    def controller_handler(self, req):
        if req.user_cmd.command=='start':
            print("CENTRAL NODE: Requested delivery UAV system start.")
            self.mtx_started.acquire()
            if self.started:
                print("CENTRAL NODE: System is already started.")
                self.mtx_started.release()
                return False
            self.mtx_started.release()
            
            self.start_sys()
            print("CENTRAL NODE: System has started.")
            
            self.mtx_started.acquire()
            self.started = True
            self.vc_started.notify_all()
            self.mtx_started.release()
            return True

        elif req.user_cmd.command=='auto':
            print("CENTRAL NODE: Requested auto mode.")

            # Espero a que el sistema este iniciado
            self.mtx_started.acquire()
            if not self.started:
                print("CENTRAL NODE: Waiting for system to start")
                while not self.started:
                    self.vc_started.wait()

            # Compruebo que el UAV no este siguiendo una trayectoria. Si no la sigue, ejecuto el auto. Si ya la sigue, rechazo el request.
            self.mtx_travel.acquire()
            if self.travel:
                print("CENTRAL NODE: The UAV has already a goal defined.")
                self.mtx_started.release() 
                return False
            
            self.travel=True # El UAV ESTA VIAJANDO
            self.mtx_travel.release()
            self.mtx_started.release()

            # Ejecuto el codigo
            self.goal[0]=req.user_cmd.x
            self.goal[1]=req.user_cmd.y
            self.goal[2]=req.user_cmd.z

            self.auto_mode(self.goal)
            print("CENTRAL NODE: Auto mode finished.")
            self.mtx_travel.acquire()
            self.travel=False # EL UAV YA NO VIAJA
            self.vc_travel.notify_all()
            self.mtx_travel.release()
            return True

            
        elif req.user_cmd.command=='gohome':
            print("CENTRAL NODE: Requested back to home point.")

            # Espero a que el sistema este iniciado
            self.mtx_started.acquire()
            if not self.started:
                print("CENTRAL NODE: Waiting for system to start")
                while not self.started:
                    self.vc_started.wait()

            # Compruebo que el UAV no este siguiendo una trayectoria. Si no la sigue, ejecuto el auto. Si ya la sigue, rechazo el request.
            self.mtx_travel.acquire()
            if self.travel:
                print("CENTRAL NODE: The UAV has already a goal defined.")
                while self.travel:
                    self.vc_travel.wait()
            self.travel=True # El UAV ESTA VIAJANDO
            self.mtx_travel.release()
            self.mtx_started.release()

            # Ejecuto el codigo
            self.gohome()
            print("CENTRAL NODE: Home point reached.")

            self.mtx_travel.acquire()
            self.travel=False # EL UAV YA NO VIAJA
            self.vc_travel.notify_all()
            self.mtx_travel.release()
            return True

        elif req.user_cmd.command=='idle':
            print("CENTRAL NODE: Requested idle mode.")

            # Espero a que el sistema este iniciado
            self.mtx_started.acquire()
            if not self.started:
                print("CENTRAL NODE: Waiting for system to start")
                while not self.started:
                    self.vc_started.wait()

            # Compruebo que el UAV no este siguiendo una trayectoria. Si no la sigue, idle es redundante y se rechaza. Si ya la sigue, acepto el request.
            self.mtx_travel.acquire()
            if not self.travel:
                print("CENTRAL NODE: The UAV is already idle.")
                self.mtx_travel.release()
                self.mtx_started.release()
                return False
            self.mtx_travel.release()
            self.mtx_started.release()

            # Ejecuto el codigo
            self.idle_mode()
            print("CENTRAL NODE: Idle mode set.")

            self.mtx_travel.acquire()
            self.travel=False # EL UAV YA NO VIAJA
            self.vc_travel.notify_all()
            self.mtx_travel.release()
            return True

        elif req.user_cmd.command=='drop':
            print("CENTRAL NODE: Requested charge drop.")

            # Espero a que el sistema este iniciado
            self.mtx_started.acquire()
            if not self.started:
                print("CENTRAL NODE: Waiting for system to start")
                while not self.started:
                    self.vc_started.wait()

            # Compruebo que el UAV siga teniendo la carga. Si no la tiene, drop no tiene sentido.
            self.mtx_charge.acquire()
            if not self.charge:
                print("CENTRAL NODE: The UAV has already dropped the charge.")
                self.mtx_charge.release()
                self.mtx_started.release()
                return False
            self.mtx_charge.release()
            self.mtx_started.release()

            # Ejecuto el codigo
            self.drop_charge()
            print("CENTRAL NODE: Charge dropped.")
            return True

        else:
            print("CENTRAL NODE: Incorrect command.")
            return False


    def start_server(self):
        s = rospy.Service('uav_user_interface', user_interface, self.controller_handler)
        print("CENTRAL NODE: User Interface ready.")
        rospy.spin()

if __name__ == "__main__":
    rospy.init_node('central_node')
    user_server = user_interface_server()
    # Esperamos a que el sistema este listo
    # Iniciamos el servidor
    user_server.start_server()
