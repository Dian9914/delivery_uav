#!/usr/bin/env python

from delivery_uav.srv import gripper_srv, user_interface
from delivery_uav.msg import gripper_state
from uav_abstraction_layer.srv import GoToWaypoint, Land, TakeOff
import rospy
from threading import Lock, Condition
from service_client import service_client #esta bien, aunque marque error. Odio pylance

class user_interface_server():
    def __init__(self):
        self.home = [0, 0, 0]
        self.goal = [0, 0, 0]

        self.mtx_auto = Lock()
        self.mtx_ready = Lock()
        self.mtx_started = Lock()
        self.mtx_charge = Lock()
        self.vc_started = Condition(self.mtx_started)
        self.vc_ready = Condition(self.mtx_ready)
        self.vc_charge = Condition(self.mtx_charge)
        
        self.ready = False       #Variable que representa si el UAV esta en condicion de recibir ordenes
        self.started = False     #Variable que representa si el sistema esta inicializado y el UAV despegado
        self.charge = True       #Variable que representa si la carga esta a bordo

        self.travel = False        #Variable que representa si el UAV ya esta dirigiendose a un punto
        self.mtx_travel = Lock()
        self.vc_travel = Condition(self.mtx_travel)

        self.ual_takeoff = service_client('/ual/take_off',TakeOff)
        self.ual_goto = service_client('/ual/go_to_waypoint',GoToWaypoint)
        self.ual_land = service_client('/ual/land',Land)
        self.gripper = service_client('/del_uav/gripper_node/control',gripper_srv)


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
        #SOLTAR LA CARGA: Cuando se le llama, abre el gripper para soltar la carga
        #Devuelve True si la carga ha sido soltada, y False si ha ocurrido algun error
        print("DROP SERVICE [CN]: Starting the drop of the charge.")

        # Espero a que el UAV este estable
        self.mtx_ready.acquire()

        # Creo el request que se enviara al servicio. Abre el gripper.
        request = gripper_state()
        request.torque = 1
        request.state = 'open'
        # Envio un unico request
        response = self.gripper.single_response(request)
        # Independientemente del resultado, la carga ya se ha liberado y el UAV puede volver a moverse.
        self.mtx_ready.release() 
        # Comprobamos el resultado de nuestra llamada.
        if not response:
            print('DROP SERVICE [CN]: Error with gripper. Cant open the gripper.')
            return False

        # Creo el request que se enviara al servicio. Deja el gripper en estado neutro.
        request = gripper_state()
        request.torque = 0
        request.state = 'idle'
        # Envio un unico request
        response = self.gripper.single_response(request)
        if not response:
            print('DROP SERVICE [CN]: WARNING: cant reach gripper.')

        print('DROP SERVICE [CN]: Charge dropped succesfully')
        return True



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

            # Espero a que no haya un servicio drop ya ejecutandose
            self.mtx_charge.acquire()
            # Espero a que el sistema este iniciado
            self.mtx_started.acquire()
            if not self.started:
                print("CENTRAL NODE: Waiting for system to start")
                while not self.started:
                    self.vc_started.wait()

            # Compruebo que el UAV siga teniendo la carga. Si no la tiene, drop no tiene sentido.
            if not self.charge:
                print("CENTRAL NODE: The UAV has already dropped the charge.")
                self.mtx_charge.release()
                self.mtx_started.release()
                return False
            self.mtx_started.release()

            # Ejecuto el codigo
            response = self.drop_charge()
            # Compruebo si ha sido un exito
            if response: 
                print("CENTRAL NODE: Charge dropped.")
                self.charge=False # LA CARGA YA NO ESTA A BORDO
                self.vc_charge.notify_all()
            else:
                print("CENTRAL NODE: Error with gripper. Cant drop the charge.")
            self.mtx_charge.release() #libero el servicio drop

            return response

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
