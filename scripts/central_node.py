#!/usr/bin/env python

from delivery_uav.srv import gripper_srv, user_interface, planner_srv, goto_srv
from delivery_uav.msg import gripper_state, waypoint, planner_route
from uav_abstraction_layer.srv import GoToWaypoint, Land, TakeOff
from uav_abstraction_layer.msg import State
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped

import rospy
from threading import Lock, Condition
from service_client import service_client #esta bien, aunque marque error. 

class user_interface_server():
    def __init__(self):
        # Inicializacion de los parametros del sistema
        self.home = [0, 0, 0]   #punto en el que se ejecuta start en coordenadas mapa (punto inciial)
        self.trayectory = []        #matriz donde se guardara la trayectoria dada por el planner
        self.pose = [0, 0, 0]   #posicion del UAV usada por UAL. Idealmente, estara en coordenadas mapa.

        self.mtx_auto = Lock()
        self.mtx_ready = Lock()
        self.mtx_started = Lock()
        self.mtx_charge = Lock()
        #variables de condicion
        self.vc_started = Condition(self.mtx_started)
        self.vc_ready = Condition(self.mtx_ready)
        
        #variables bandera
        self.ready = False       #Variable que representa si el UAV esta en condicion de recibir ordenes
        self.started = False     #Variable que representa si el sistema esta inicializado y el UAV despegado
        self.charge = True       #Variable que representa si la carga esta a bordo
        self.ual_state = 0   #Bandera que informa del estado de ual y de si es posible despegar o controlar
        # 0: ual no esta listo, 1: se puede lanzar start, 2: se puede controlar

        # sistema para evitar que el uav reciba instrucciones contradictorias
        self.travel = False        #Variable que representa si el UAV ya esta dirigiendose a un punto
        self.mtx_travel = Lock()
        self.vc_travel = Condition(self.mtx_travel)

        # inicializacion de los clientes para los distintos servicios a usar
        self.takeoff = service_client('/ual/take_off',TakeOff)
        #CUIDADO; DESCOMENTAR TAMBIEN EN AUTO_MODE LINEA 215 aprox
        #self.goto = service_client('/ual/go_to_waypoint',GoToWaypoint)
        self.goto = service_client('/del_uav/goto',goto_srv)
        self.land = service_client('/ual/land',Land)
        self.gripper = service_client('/del_uav/gripper_cmd',gripper_srv)
        self.planner = service_client('/del_uav/planner',planner_srv)

    def ual_state_checker(self,data):
        # al leer el estado, comprobamos que el estado sea landed armed para poder hacer start
        # o flying auto para poder controlar
        if data.state==2 and self.ual_state==0:
            self.ual_state=1
            print("CENTRAL NODE: UAL ready for take off.")
        elif data.state==5 and self.ual_state==1:
            self.ual_state=0
            print("CENTRAL NODE: UAL is not ready for take off.")
        elif data.state==4 and self.ual_state==1:
            self.ual_state=2
            print("CENTRAL NODE: UAL is ready for taking waypoints.")
        elif data.state!=4 and self.ual_state==2:
            self.ual_state=0
            print("CENTRAL NODE: UAL has either started landing or failed.")


    def subscriber_callback(self, data):
        self.pose = [data.pose.position.x, data.pose.position.y, data.pose.position.z]
        self.pose[0] = round(self.pose[0],1)
        self.pose[1] = round(self.pose[1],1)
        self.pose[2] = round(self.pose[2],1)

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


    def start_sys(self):
        #Inicio del sistema, despega el UAV, guarda la posicion inicial como HOME y permite el resto de llamadas
        print("START SERVICE [CN]: Starting the delivery UAV system.")

        self.mtx_ready.acquire() #el uav se va a mover, no nos interesa que ningun otro hilo intente moverlo

        # Al principio debemos cerrar el gripper para sujetar bien la carga
        # Creo el request que se enviara al servicio. Cierra el gripper.
        request = gripper_state()
        request.torque = 1
        request.state = 'close'
        # Envio un unico request
        response = self.gripper.single_response(request)
        # Comprobamos el resultado de nuestra llamada.
        if not response:
            print('START SERVICE [CN]: Error with gripper. Cannot close the gripper.')
            return False

        request = TakeOff._request_class() #la request que se envia al servicio takeoff
        request.height = 3.0              # En una primera aproximacion al problema, asumiremos que siempre es seguro elevarse 3m
        # mas adelante, se llamara al planner para preguntarle cual es la altura segura para elevarse

         # debemos esperar a que UAL este listo para despegar. Para ello, usaremos rospy.sleep
        wait = rospy.Rate(2) # vamos a esperar la condicion con una frecuencia de 2Hz, es decir, que el proceso estara
        # bloqueado durante 0.5 segundos aproximadamente antes de comprobar el estado de UAL
        # Para controlar no estar demasiado tiempo esperando, usamos un contador
        t = 0
        bandera_state = False #bandera para evitar falsos positivos
        while not (self.ual_state == 1 and bandera_state): 
            t=t+1
            if self.ual_state == 1:
                bandera_state = True
            else:
                bandera_state = False
            if t > 100:
                print('START SERVICE [CN]: UAL is taking more time than espected...')
                entrada = raw_input('START SERVICE [CN]: Do you want to keep trying? [S/n] ->')
                if entrada == 'n':    # Si la respuesta es no, el sistema abortara el inicio y devolvera False
                    print('START SERVICE [CN]: Aborting takeoff. System start aborted')
                    self.mtx_ready.release()
                    return False
                else:
                    print('START SERVICE [CN]: Central node will keep waiting.')
                    t=0
                    continue

            wait.sleep()

        # llamo al servicio takeoff
        response = self.takeoff.single_response(request)

        # se evalua la respuesta
        # Mientras que la respuesta sea un error, el sistema seguira preguntando al usuario si quiere reintentar el inicio
        while not response:     
            print('START SERVICE [CN]: Error with takeoff.')
            entrada = raw_input('START SERVICE [CN]: Try again? [S/n] -> ')     #existe en python2 para obtener texto por teclado
            if entrada == 'S':    # Si la respuesta es si, el sistema esp erara a que el servicio este de nuevo disponible y lo llamara
                self.takeoff.is_avalible()
                response = self.takeoff.single_response(request)
            elif entrada == 'n':    # Si la respuesta es no, el sistema abortara el inicio y devolvera False
                print('START SERVICE [CN]: Aborting takeoff. System start aborted')
                self.mtx_ready.release()
                return False
            else:
                print('START SERVICE [CN]: Unrecognised input.')
                continue
        

        # debemos esperar a haber llegado al punto para hacer la siguiente llamada. Para ello, usaremos rospy.sleep
        wait = rospy.Rate(2) # vamos a esperar la condicion con una frecuencia de 2Hz, es decir, que el proceso estara
        # bloqueado durante 0.5 segundos aproximadamente antes de comprobar que hemos llegado al punto
        while not (self.ual_state == 2): 
            #mientras no hayamos llegado a la altura deseada y el estado de UAL no sea flying auto
            wait.sleep()
        # Registramos la posicion UNA VEZ ESTAMOS VOLANDO como punto home
        # Esto es importante porque al planificador nunca debemos darle como objetivo un punto en el suelo
        # Ya que podria planificar un camino potencialmente peligroso demasiado cercano al suelo
        self.home = self.pose

        print('START SERVICE [CN]: UAV is now flying and ready.')
        self.mtx_ready.release()        #el sistema esta listo para que otros hilos muevan al UAV
        return True

    def auto_mode(self, goal):
        #MODO AUTOMATICO: Principal modo de funcionamiento, el UAV se desplaza a un punto dado
        print("AUTO MODE [CN]: Starting the auto mode. The goal set is [%d, %d, %d]."%(goal[0],goal[1],goal[2]))

        
        #SOLO DESCOMENTAR SI EL SERVICO DEL PLANNER FUNCIONA Y HAS DESCOMENTADO ARRIBA SU HANDLER
        # PRIMERO LLAMARIAMOS AL SERVICIO DEL PLANNER QUE NOS SUMINISTRE LA TRAYECTORIA
        request = planner_srv._request_class()

        request.start.xyz[0]=self.pose[0]
        request.start.xyz[1]=self.pose[1]
        request.start.xyz[2]=self.pose[2]
        request.goal.xyz[0]=goal[0]
        request.goal.xyz[1]=goal[1]
        request.goal.xyz[2]=goal[2]

        
        response = self.planner.single_response(request)
        if not response:
            print('AUTO MODE [CN]: Error with Planner service. Aborting travel')
            return False
        else:
            print('AUTO MODE [CN]: Succesfull Planner service call.')

        self.trayectory = []
        for i in range(0,len(response),3):
            self.trayectory.append([response[i],response[i+1],response[i+2]])

        # IMPORTANTE:
        # COMO NO DISPONEMOS DE PLANNER, SUMINISTRAMOS UNA TRAYECTORIA INVENTADA
        '''self.trayectory = [[-1,-1,3],[-2,-2,3],[-2,-2,4],[-3,-3,4],[-4,-4,4],[-3,-3,4],[-2,-2,4],[-2,-2,3],[-2,-3,3]]'''

        # En este caso vamos a llamar al mismo servicio de forma recurrente, por lo que es interesante usar una conexion persistente con el servicio
        # Por tanto, inicializaremos la conexion antes de entrar al bucle
        response = self.goto.persistent_init()
        # Es importante cerrar mas adelante esta conexion
        # Evaluamos la respuesta de la inicializacion
        if not response:
            print('AUTO MODE [CN]: Error with GoToWaypoint service. Aborting travel')
            return False
        else:
            print('AUTO MODE [CN]: Started persistent GoToWaypoint service.')

        # Ademas, generamos el mensaje que pasaremos al servicio ya que el tipo de este no va a cambiar durante la ejecucion
        # CUIDADO AL CAMBIAR DE SERVICIO
        # request = GoToWaypoint._request_class()
        request = goto_srv._request_class()

        # comienza el bucle de viaje
        for waypoint in self.trayectory:
            # primero adquirimos el mutex, para asegurarnos de que ningun otro hilo manda ordenes al UAV durante el movimiento
            self.mtx_ready.acquire()
            # comprobamos que ningun hilo idle haya abortado el viaje
            self.mtx_travel.acquire()
            if not self.travel:
                print("AUTO MODE [CN]: Travel aborted")
                self.mtx_travel.release()
                self.mtx_ready.release()
                break
            self.mtx_travel.release()
            # actualizamos los valores de request para el siguiente waypoint
            request.waypoint.pose.position.x = waypoint[0]
            request.waypoint.pose.position.y = waypoint[1]
            request.waypoint.pose.position.z = waypoint[2]

            #llamamos al servicio para comenzar el movimiento
            response = self.goto.persistent_response(request)
            while not response:     
                print('AUTO MODE [CN]: Error with GoToWaypoint.')
                entrada = raw_input('AUTO MODE [CN]: Try again? [S/n] -> ')     #existe en python2 para obtener texto por teclado
                if entrada == 'S':    # Si la respuesta es si, reiniciamos la conexion y reintentamos
                    self.goto.persistent_close()
                    self.goto.persistent_init()
                    response = self.goto.persistent_response(request)
                elif entrada == 'n':    # Si la respuesta es no, el sistema abortara el inicio y devolvera False
                    print('AUTO MODE [CN]: Error reaching waypoint [%d, %d, %d].'%(waypoint[0],waypoint[1],waypoint[2]))
                    print('AUTO MODE [CN]: Aborting auto_mode.')
                    self.mtx_ready.release()
                    break
                else:
                    print('AUTO MODE [CN]: Unrecognised input.')
                    continue
            # debemos esperar a haber llegado al punto para hacer la siguiente llamada. Para ello, usaremos rospy.sleep
            wait = rospy.Rate(5) # vamos a esperar la condicion con una frecuencia de 2Hz, es decir, que el proceso estara
            # bloqueado durante 0.2 segundos aproximadamente antes de comprobar que hemos llegado al punto
            while not (abs(self.pose[0] - waypoint[0])<0.5 and abs(self.pose[1] - waypoint[1])<0.5 and abs(self.pose[2] - waypoint[2])<0.5):
                wait.sleep()

            # Una vez hemos hecho la orden, abrimos el mutex para permitir que otros hilos den ordenes al UAV
            self.mtx_ready.release()

            # Comprobamos el resultado de nuestra llamada a servicio.
            print('AUTO MODE [CN]: Succesfully reached waypoint [%d, %d, %d].'%(waypoint[0],waypoint[1],waypoint[2]))
            #print('DEBUG AUTO MODE [CN]: Current position is [%f, %f, %f].'%(self.pose[0],self.pose[1],self.pose[2]))

        # Esperamos a llegar al destino y estabilizarnos, las tolerancias aqui son mas finas
        t=0
        while not (abs(self.pose[0] - waypoint[0])<0.3 and abs(self.pose[1] - waypoint[1])<0.3 and abs(self.pose[2] - waypoint[2])<0.5) and t<10:
            if abs(self.pose[0] - goal[0])<0.3 and abs(self.pose[1] - goal[1])<0.3 and abs(self.pose[2] - goal[2])<0.5:
                t=t+1
            else:
                t=0
            wait.sleep()
        
        if abs(self.pose[0] - goal[0])<0.3 and abs(self.pose[1] - goal[1])<0.3 and abs(self.pose[2] - goal[2])<0.5:
            print('AUTO MODE [CN]: Succesfully reached goal [%d, %d, %d].'%(goal[0],goal[1],goal[2]))
            result = True
        else:
            print('AUTO MODE [CN]: Could not reach goal. Instead, arrived at [%f, %f, %f].'%(self.pose[0],self.pose[1],self.pose[2]))
            result = False
            
        # Cerramos la conexion
        response = self.goto.persistent_close()
        if not response:
            print('AUTO MODE [CN]: WARNING: Error closing GoToWaypoint service.')

        return result

    def idle_mode(self):
        #MODO IDLE: Se cancela cualquier ruta y se ordena al UAV que permanezca inmovil.
        print("IDLE MODE [CN]: Switching to idle mode.")

        # primero debemos esperar a que el UAV este estatico en el aire y listo para recibir ordenes
        self.mtx_ready.acquire()
        # a continuacion, cambiamos el estado del UAV poniendo a false la variable travel
        self.mtx_travel.acquire()
        self.travel = False
        self.vc_travel.notify()
        self.mtx_travel.release()
        self.mtx_ready.release()

        return True
        

    def drop_charge(self):
        #SOLTAR LA CARGA: Cuando se le llama, abre el gripper para soltar la carga
        #Devuelve True si la carga ha sido soltada, y False si ha ocurrido algun error
        print("DROP SERVICE [CN]: Starting charge drop off.")

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
            print('DROP SERVICE [CN]: Error with gripper. Cannot open the gripper.')
            return False

        #espera un segundo para que el gripper abra
        duerme=rospy.rate(1)
        duerme.sleep()

        # Creo el request que se enviara al servicio. Deja el gripper en estado neutro.
        request = gripper_state()
        request.torque = 0
        request.state = 'idle'
        # Envio un unico request
        response = self.gripper.single_response(request)
        if not response:
            print('DROP SERVICE [CN]: WARNING: cannot reach gripper.')

        print('DROP SERVICE [CN]: Charge dropped succesfully')
        return True



    def gohome(self):
        #Vuelta a casa: va hacia el punto inicial y suelta la carga ahi
        print("GOHOME SERVICE [CN]: Starting home path mode.")
        print("GOHOME SERVICE [CN]: Requesting auto mode service with goal set in home.")

        response = self.auto_mode(self.home)

        if not response:
            print("GOHOME SERVICE [CN]: Auto mode aborted.")
            return False

        print("GOHOME SERVICE [CN]: Home reached. Requesting landing")
        self.mtx_ready.acquire()
        request = Land._request_class()
        response = self.land.single_response(request)
        while not response:     
            print('GOHOME SERVICE  [CN]: Error with landing.')
            entrada = raw_input('GOHOME SERVICE  [CN]: Try again? [S/n] ->')     #existe en python2 para obtener texto por teclado
            if entrada == 'S':    # Si la respuesta es si, reiniciamos la conexion y reintentamos
                self.land.is_avalible()
                response = self.land.single_response(request)
            elif entrada == 'n':    # Si la respuesta es no, el sistema abortara el inicio y devolvera False
                print('GOHOME SERVICE  [CN]: Aborting landing.')
                self.mtx_ready.release()
                break
            else:
                print('GOHOME SERVICE  [CN]: Unrecognised input.')
                continue
        
        self.mtx_ready.release()


        return True 


    def controller_handler(self, req):
        if req.user_cmd.command=='start':
            print("CENTRAL NODE: Requested delivery UAV system start.")
            self.mtx_charge.acquire() #vamos a trabajar con la carga
            self.mtx_started.acquire()
            if self.started:
                print("CENTRAL NODE: System is already started.")
                self.mtx_started.release()
                self.mtx_charge.release() 
                return False
            
            response = self.start_sys()
            
            if response:
                print("CENTRAL NODE: System has started.")
                self.started = True
                self.vc_started.notify_all()
                self.charge = True
                print("CENTRAL NODE: Charge loaded.")
            else:
                print("CENTRAL NODE: System start failed.")
            self.mtx_charge.release() 
            self.mtx_started.release()
            return response

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
                self.mtx_travel.release()
                return False
            
            self.travel=True # El UAV ESTA VIAJANDO
            self.mtx_travel.release()
            self.mtx_started.release()

            # Ejecuto el codigo
            goal = req.user_cmd.goal.xyz

            response = self.auto_mode(goal)
            print("CENTRAL NODE: Auto mode finished.")
            
            self.mtx_travel.acquire()
            if self.travel == True: # si no se ha abortado usando idle, travel seguira a True
                self.travel=False # EL UAV YA NO VIAJA
                self.vc_travel.notify()
            self.mtx_travel.release()

            return response

            
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
            response = self.gohome()


            self.mtx_travel.acquire()
            if self.travel == True: # si no se ha abortado usando idle, travel seguira a True
                self.travel=False # EL UAV YA NO VIAJA
                self.vc_travel.notify()
            self.mtx_travel.release()

            if response: 
                print("CENTRAL NODE: Home point reached.")

                # El sistema ya no esta iniciado
                self.mtx_started.acquire()
                self.started = False
                self.mtx_started.release()
            else:
                print("CENTRAL NODE: Could not reach home.")

            return response

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
            else:
                print("CENTRAL NODE: Error with gripper. Cannot drop the charge.")

            self.mtx_charge.release() #libero el servicio drop

            return response

        else:
            print("CENTRAL NODE: Incorrect command.")
            return False

    def initial_lap(self):
        print("CENTRAL NODE: Starting initial lap for localization convergence.")
        # Este metodo debe hacer que el UAV se eleve unos metros y comience a navegar en bucle abierto, 
        # usando unicamente el lidar para evitar obstaculos, hasta que la localizacion haya convergido.
        # Aqui se plantean dos problematicas:
        # - La primera es que debemos poder pilotar en bucle abierto, sin depender del valor de UAL/pose
        #   porque este no habra convergido todavia. Los servicios de UAL NO FUNCIONARIAN CORRECTAMENTE
        # - La segunda es que debemos poder saber cuando UAL/pose ha convergido. 
        #   Podemos usar un servicio que sea llamado desde localizacion o analizar desde aqui los datos

        print("CENTRAL NODE: Localization successful.")



    def start_server(self):
        self.start_subscriber()
        s = rospy.Service('/del_uav/user_interface', user_interface, self.controller_handler)
        # IMPORTANTE:
        # antes de iniciar los servicios, debemos dar una vuelta sin objetivo claro
        # a fin de que la localizacion pueda converger
        self.initial_lap()
        # Para esto, debemos navegar usando solo la informacion del lidar, buscando no chocarnos con nada
        print("CENTRAL NODE: User Interface ready.")
        rospy.spin()

if __name__ == "__main__":
    rospy.init_node('central_node')
    user_server = user_interface_server()
    # Esperamos a que el sistema este listo
    # Iniciamos el servidor
    user_server.start_server()
