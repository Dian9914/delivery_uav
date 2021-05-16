import rospy

class service_client():
    # Clase generica para llamar a servicios. Se debe inicializar con el nombre del servicio y el srv
    def __init__(self, service_name, srv_name):
        self.service = service_name
        self.srv = srv_name
        rospy.wait_for_service(self.service)
        print('SERVICE CLIENT MANAGER: Service %s ready'%self.service)


    def single_response(self, my_param):
        # Para una unica llamada. Llama a un servicio y devuelve su respuesta
        try:
            s_service = rospy.ServiceProxy(self.service, self.srv, persistent=False)
            s_service_response = s_service(my_param)
            return s_service_response
        except rospy.ServiceException as e:
            print ('SERVICE CLIENT MANAGER: Service call failed: %s' % e)
            return False
        except TypeError as e:
            print ('SERVICE CLIENT MANAGER: Wrong parameter for request: %s' % e)
            return False


    def persistent_init(self):
        # Inicia una conexion persistente con un servicio, permitiendo multiples llamadas al mismo servicio
        try:
            self.p_service = rospy.ServiceProxy(self.service, self.srv, persistent=True)
            return True
        except rospy.ServiceException as e:
            print ('SERVICE CLIENT MANAGER: Service conection failed: %s' % e)
            return False


    def persistent_response(self, my_param):
        # Llama a un servicio con el que ya hemos iniciado una conexion persistente. IMPORTANTE: haber inicializado el servicio antes
        try:
            p_service_response = self.p_service(my_param)
            return p_service_response
        except rospy.ServiceException as e:
            print ('SERVICE CLIENT MANAGER: Service call failed: %s' % e)
            return False


    def persistent_close(self):
        # Cierra una conexion persistente con un servicio.
        try:
            self.p_service.close()
            print('SERVICE CLIENT MANAGER: Persistent conection with service %s ended'%self.service)
            return True
        except rospy.ServiceException as e:
            print ('SERVICE CLIENT MANAGER: Service conection failed: %s' % e)
            return False

    def is_avalible(self):
        # Espera a que el servicio vuelva a estar disponible
        rospy.wait_for_service(self.service)
        print('SERVICE CLIENT MANAGER: Service %s ready'%self.service)
        return True
