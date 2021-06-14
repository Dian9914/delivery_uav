#!/usr/bin/env python
"""
Created on Sun May 23 12:20:08 2021

@author: jorge
"""
import re
import rospy
import time
import os
from delivery_uav.srv import planner_srv
from delivery_uav.msg import planner_route
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
import numpy as np
from math import sqrt, pow

class lista:
    # esta clase contiene los atributos que definen la lista abierta y la cerrada
    # cada elemento de la lista sera un objeto que contiene el punto, el punto padre, y los valores de f, g y h
    punto = [0, 0,0]
    f = 0
    g = 0
    h = 0
    padre = [0, 0,0]

    def __init___(self):
        pass
class Planner:
    
    def read_pgm(self,filename, byteorder='>'):
        """Return image data from a raw PGM file as numpy array. Format specification: http://netpbm.sourceforge.net/doc/pgm.html """
        with open(filename, 'rb') as f:
            buffer = f.read()
        try:
            header, width, height, maxval = re.search(
            b"(^P5\s(?:\s*#.*[\r\n])*"
            b"(\d+)\s(?:\s*#.*[\r\n])*"
            b"(\d+)\s(?:\s*#.*[\r\n])*"
            b"(\d+)\s(?:\s*#.*[\r\n]\s)*)", buffer).groups()
        except AttributeError:
            raise ValueError("Not a raw PGM file: '%s'" % filename)
        return np.frombuffer(buffer,
                            dtype='u1' if int(maxval) < 256 else byteorder+'u2',
                            count=int(width)*int(height),
                            offset=len(header)
                            ).reshape((int(height), int(width)))
    def __init__(self):
        self.inicio=time.time()
        ruta=os.path.dirname(os.path.abspath(__file__))
        # Empezamos comprobando si hemos cambiado el valor de k_seguridad y en caso contrario utilizamos el valor por defecto
        if rospy.has_param('~k_seguridad'):
            k_seguridad = rospy.get_param('~k_seguridad')
        else:
            k_seguridad = 3
        # Leemos los tres mapas que vamos a utilizar para formar el voxelgrid
        imagen0 = self.read_pgm(ruta+"/../maps/mapa0_0.5.pgm", byteorder='<')
        imagen3 = self.read_pgm(ruta+"/../maps/mapa3_0.5.pgm", byteorder='<')
        imagen6 = self.read_pgm(ruta+"/../maps/mapa6_0.5.pgm", byteorder='<')
        #Generamos una matriz de ceros donde guardaremos el mapa
        self.map=np.zeros((100,100,8))
        #Ahora comprobamos los mapas leidos y en caso de que alguna coordenada este libre(254) se copia en nuestro mapa, esto lo
        #hacemos utilizando el mapa 0 para las alturas 0 y 1 metro, el mapa 3 para 2,3 y 4 y el mapa 6 para la 5 y 6, con esto 
        #se consigue 1 metro aproximado de distancia de seguridad en Z
        self.map[imagen0==254,0:2]=254
        self.map[imagen3==254,2:5]=254
        self.map[imagen0==254,5:7]=254
        #Ahora vamos a anadir la distancia de seguridad en X e Y, para ello vamos a iterar por las diferentes alturas de la matriz
        #en caso de que algun voxel sea un obstaculo se maracara como obstaculo todos sus abyacentes en un rango de K_seguridad
        for i in range(5,95,1) :
            for j in range(5,95,1):
                if imagen0[i,j]!=254:
                    for k in range(k_seguridad):
                            self.map[i+k,j,0:2]=0
                            self.map[i+k,j+k,0:2]=0
                            self.map[i,j+k,0:2]=0
                            self.map[i-k,j,0:2]=0
                            self.map[i-k,j-k,0:2]=0
                            self.map[i,j-k,0:2]=0
                            self.map[i-k,j+k,0:2]=0
                            self.map[i+k,j-k,0:2]=0                    
                if imagen3[i,j]!=254:
                    for k in range(k_seguridad):
                        self.map[i+k,j,2:6]=0
                        self.map[i+k,j+k,2:5]=0
                        self.map[i,j+k,2:5]=0
                        self.map[i-k,j,2:5]=0
                        self.map[i-k,j-k,2:5]=0
                        self.map[i,j-k,2:5]=0
                        self.map[i-k,j+k,2:5]=0
                        self.map[i+k,j-k,2:5]=0        
                if imagen6[i,j]!=254:
                    for k in range(k_seguridad):
                        self.map[i+k,j,5:7]=0
                        self.map[i+k,j+k,5:7]=0
                        self.map[i,j+k,5:7]=0
                        self.map[i-k,j,5:7]=0
                        self.map[i-k,j-k,5:7]=0
                        self.map[i,j-k,5:7]=0
                        self.map[i-k,j+k,5:7]=0
                        self.map[i+k,j-k,5:7]=0
            
        self.height = 100
        self.width = 100
        self.resolution = 0.5
        
    def euclidean_distance(self, posicion_actual, posicion_final):
        #Distancia euclidea entre posicion actual y final
        #Se calcula mediante las coordenadas x e y de los puntos, y obteniendo la hipotenusa del triangulo resultante
        return sqrt(pow((posicion_final[0] - posicion_actual[0]), 2) +
                    pow((posicion_final[1] - posicion_actual[1]), 2) +
                    pow((posicion_final[2] - posicion_actual[2]),2))

    def manhattan_distance(self, node_initial, node_final):
        #Distancia Manhattan: Se define como la suma de la distancia en X, la distancia en Y y la distancia en Z entre los dos nodos
        return (abs(node_final[0]-node_initial[0]) + abs(node_final[1]-node_initial[1])+abs(node_final[2]-node_initial[2]))
    
    def check_in_list(self, elemento, lista_b):
        #Comprueba si un elemento del tipo lista (objeto) se encuentra dentro de un array de objetos lista.
        #Mediante un bucle recorremos la lista hasta encontrar una coincidencia, en ese caso devolvemos 1 y escribimos en 
        #variables globales el lugar de la lista donde lo ha encontrado y el elemento en si.
        for index in range(len(lista_b)):
            if elemento.punto == lista_b[index].punto:
                self.index_found = index
                self.elemento_found = lista_b[index]
                return 1
        return 0
    def compute_path(self, data):
        """Calcula la ruta entre el nodo inicial y el final"""
        # DEFINICIONES
        # La lista abierta incluye los nodos que hemos encontrado y no hemos procesado
        # Sera un array de objetos lista, cada uno contiene puntos y sus propiedades
        self.lista_abierta = []

        # La lista cerrada incluye los nodos que hemos visitado y procesado
        # Tambien es un array de objetos lista
        self.lista_cerrada = []

        # La celda actual sera la que procesemos en cada instante. Es un unico objeto del tipo lista
        self.celda_actual = lista()

        # Estas variables son para obtener elementos de una lista
        self.index_found = 0
        self.elemento_found = lista()
        
        #Valores de offset, depende del tamano del mapa y de la posicion inicial que queramos
        self.offsX=50
        self.offsY=50
        
        start=data.start.xyz
        goal=data.goal.xyz
        
        # Paso previo: Es necesario hacer un cambio de coordenadas, dado que el mapa procesado tiene su origen en el elemento 
        # arriba a la izquierda y en el simulador, el origen de coordenadas es en el centro
        # Se ha obtenido que:
        start_cell_map = [int(round(start[0]/0.5)+self.offsX), int(self.offsY-round(start[1]/0.5)),int(start[2])] 
        goal_cell_map = [int(round(goal[0]/0.5)+self.offsX), int(self.offsY-round(goal[1]/0.5)),int(goal[2])]
         # Paso 0: Determinamos los atributos de la celda actual y se introduce el punto en la lista abierta
        self.celda_actual.punto = [start_cell_map[0],start_cell_map[1],start_cell_map[2]]
        self.celda_actual.padre = [start_cell_map[0],start_cell_map[1],start_cell_map[2]]
        self.celda_actual.h = self.euclidean_distance(start_cell_map, goal_cell_map)
        self.celda_actual.f = self.celda_actual.h
        self.lista_abierta.append(self.celda_actual) # append nos permite introducir elementos en vectores
        #Miramos si el punto de destino es valido
        if (self.map[goal_cell_map[1],goal_cell_map[0],goal_cell_map[2]]==254):
            meta=1
            print('PLANNER: PLANNING PATHING, THIS MAY TAKE A WHILE')
        else:
            meta=0
            print('PLANNER_ERROR: GOAL [%.2f, %.2f, %.2f] IS NOT A VALID POINT, PLEASE TRY AGAIN'%(goal[0],goal[1],goal[2]))
        #En caso de ser valido y mientras la lista abierta no este vacia iteramos
        while meta==1 and not self.lista_abierta==[]:
            # Paso 1: Sacar el primer elemento de la lista abierta, y meterlo en la cerrada
            # El primer elemento de la lista abierta sera aquel de menor f (coste)
            self.lista_cerrada.append(self.lista_abierta[0])  
            self.celda_actual = self.lista_abierta[0]
            self.lista_abierta.pop(0)  # pop nos permite eliminar una fila de un array
            
            
            # Paso 2: Comprobar si el punto que se esta procesando es el destino
            if (self.celda_actual.punto == [goal_cell_map[0],goal_cell_map[1],goal_cell_map[2]]): break  # me salgo del bucle

            
            # Paso 3: Calcular celdas vecinas a la actual              
            # definir vector vecinos y vecinos filtrados (alternativa, usar clear)
            # Consideracion: Los vecinos diagonales no se cuentan
            lista_vecinos = []
            
            # creamos el array del tipo lista, y solo metemos dentro los puntos vecinos (el calculo de parametros vendra despues)
            for index in range(26):
                lista_vecinos.append(lista())

            # calculamos los vecinos como los adyacentes
            lista_vecinos[0].punto = [self.celda_actual.punto[0]+1, self.celda_actual.punto[1]  ,self.celda_actual.punto[2]]
            lista_vecinos[1].punto = [self.celda_actual.punto[0],   self.celda_actual.punto[1]+1 ,self.celda_actual.punto[2]]
            lista_vecinos[2].punto = [self.celda_actual.punto[0]-1, self.celda_actual.punto[1]   ,self.celda_actual.punto[2]]
            lista_vecinos[3].punto = [self.celda_actual.punto[0],   self.celda_actual.punto[1]-1 ,self.celda_actual.punto[2]]

            lista_vecinos[4].punto = [self.celda_actual.punto[0]+1, self.celda_actual.punto[1]+1 ,self.celda_actual.punto[2]]
            lista_vecinos[5].punto = [self.celda_actual.punto[0]+1, self.celda_actual.punto[1]-1 ,self.celda_actual.punto[2]]
            lista_vecinos[6].punto = [self.celda_actual.punto[0]-1, self.celda_actual.punto[1]+1 ,self.celda_actual.punto[2]]
            lista_vecinos[7].punto = [self.celda_actual.punto[0]-1, self.celda_actual.punto[1]-1 ,self.celda_actual.punto[2]]
            
            lista_vecinos[8].punto = [self.celda_actual.punto[0]+1, self.celda_actual.punto[1]  ,self.celda_actual.punto[2]-1]
            lista_vecinos[9].punto = [self.celda_actual.punto[0],   self.celda_actual.punto[1]+1 ,self.celda_actual.punto[2]-1]
            lista_vecinos[10].punto = [self.celda_actual.punto[0]-1, self.celda_actual.punto[1]   ,self.celda_actual.punto[2]-1]
            lista_vecinos[11].punto = [self.celda_actual.punto[0],   self.celda_actual.punto[1]-1 ,self.celda_actual.punto[2]-1]

            lista_vecinos[12].punto = [self.celda_actual.punto[0]+1, self.celda_actual.punto[1]+1 ,self.celda_actual.punto[2]-1]
            lista_vecinos[13].punto = [self.celda_actual.punto[0]+1, self.celda_actual.punto[1]-1 ,self.celda_actual.punto[2]-1]
            lista_vecinos[14].punto = [self.celda_actual.punto[0]-1, self.celda_actual.punto[1]+1 ,self.celda_actual.punto[2]-1]
            lista_vecinos[15].punto = [self.celda_actual.punto[0]-1, self.celda_actual.punto[1]-1 ,self.celda_actual.punto[2]-1]
            
            lista_vecinos[16].punto = [self.celda_actual.punto[0]+1, self.celda_actual.punto[1]  ,self.celda_actual.punto[2]+1]
            lista_vecinos[17].punto = [self.celda_actual.punto[0],   self.celda_actual.punto[1]+1 ,self.celda_actual.punto[2]+1]
            lista_vecinos[18].punto = [self.celda_actual.punto[0]-1, self.celda_actual.punto[1]   ,self.celda_actual.punto[2]+1]
            lista_vecinos[19].punto = [self.celda_actual.punto[0],   self.celda_actual.punto[1]-1 ,self.celda_actual.punto[2]+1]

            lista_vecinos[20].punto = [self.celda_actual.punto[0]+1, self.celda_actual.punto[1]+1 ,self.celda_actual.punto[2]+1]
            lista_vecinos[21].punto = [self.celda_actual.punto[0]+1, self.celda_actual.punto[1]-1 ,self.celda_actual.punto[2]+1]
            lista_vecinos[22].punto = [self.celda_actual.punto[0]-1, self.celda_actual.punto[1]+1 ,self.celda_actual.punto[2]+1]
            lista_vecinos[23].punto = [self.celda_actual.punto[0]-1, self.celda_actual.punto[1]-1 ,self.celda_actual.punto[2]+1]
            
            lista_vecinos[24].punto = [self.celda_actual.punto[0], self.celda_actual.punto[1],self.celda_actual.punto[2]+1]
            lista_vecinos[25].punto = [self.celda_actual.punto[0], self.celda_actual.punto[1] ,self.celda_actual.punto[2]-1]
            
            # Paso 4.1: Filtrado de vecinos
            i=0
            # para cada vecino
            while i < len(lista_vecinos): 
                # primero comprobamos si el punto seleccionado se corresponde con un punto en el mapa
                # OJO: la coordenada X me da la fila, que es el valor de Y del punto
                # si el punto es [3,12] yo quiero la columna 3, y la fila 12
                if self.map[lista_vecinos[i].punto[1],lista_vecinos[i].punto[0],lista_vecinos[i].punto[2]] == 254:
                    # Si no es un obstaculo, comprobamos si esta en la lista cerrada (es decir, si ya lo hemos procesado, no lo procesamos de nuevo)
                    if self.check_in_list(lista_vecinos[i], self.lista_cerrada):
                        # eliminamos el elemento de la lista, y decrementamos en 1 el contador para escoger correctamente el siguiente elemento
                        lista_vecinos.pop(i)
                        i=i-1
                else:
                    lista_vecinos.pop(i)
                    i=i-1

                # pasamos al siguiente elemento
                i=i+1
            # lista_vecinos ahora contiene los vecinos que van a ser procesados
            # Paso 4.2: Procesado de vecinos
            # Para cada vecino... (bucle autoindexado)
            for point in lista_vecinos:
                # Calculo el coste g(n) nuevo, que sera el coste g de la celda actual mas la distancia Manhattan
                # entre la celda actual y la celda vecina (por los vecinos elegidos, siempre va a valer 1, porque de la celda actual a la vecina siempre
                # hay 1 de distancia)
                costeNuevo = self.celda_actual.g + self.euclidean_distance(self.celda_actual.punto, 
                                                                         point.punto)
                # Comprobamos si el punto se encuentra ya en la lista abierta
                temp = self.check_in_list(point, self.lista_abierta)

                # Si resulta que el nuevo coste de este vecino es mejor que el que ya tiene (si esta ya en la lista abierta), 
                # o que el punto no se encuentra en la lista abierta
                if ((costeNuevo < self.elemento_found.g) or (temp == 0)):
                    # calculamos los parametros del punto
                    point.h = self.euclidean_distance(point.punto, [goal_cell_map[0],goal_cell_map[1],goal_cell_map[2]])
                    point.g = costeNuevo
                    point.f = point.h + point.g
                    point.padre = self.celda_actual.punto

                    # comprobamos si el nodo esta en la lista abierta, y si no lo metemos
                    if not(self.check_in_list(point, self.lista_abierta)):
                        self.lista_abierta.append(point)

                    else:  
                        # si no actualizamos los valores para el punto de la lista abierta
                        self.lista_abierta[self.index_found] = point

            #print('paso42')
            # Paso 5: Ordenar la lista abierta
            # Buscaremos ordenarla por el valor de f de menor a mayor. En caso de empate se coge el valor de h
            self.lista_abierta.sort(key=lambda var: (var.f, var.h)) 
            #print(self.lista_abierta)
        ## --FIN DEL BUCLE--
        # Paso 6: ahora tenemos que obtener los puntos de forma regresiva
        # para ello partimos del punto final, comprobamos su padre, y lo buscamos en la lista cerrada
        # luego ese punto pasa a ser el siguiente y asi sucesivamente
        if meta==1:
            path = [] # vector con los puntos por los que pasaremos
            
            # mientras que 
            while self.celda_actual.punto != start_cell_map:
                path.append(self.celda_actual.punto) # meto el punto actual
                # busco el padre en la lista para ver el punto, primero pongo que el padre sea el punto de la celda actual
                self.celda_actual.punto = self.celda_actual.padre

                # busco en la lista y obtengo el elemento de lista cerrada donde se encuentra (en index_found)
                self.check_in_list(self.celda_actual, self.lista_cerrada) # obtengo self.index_found
                self.celda_actual = self.lista_cerrada[self.index_found] # me cambio a este punto

                # por ultimo, la lista de puntos a seguir sera la inversa de la obtenida
            path.reverse()
            for i in range(len(path)):
                path[i] = [round(((path[i][0]-self.offsX)*0.5),2),round(((self.offsY-path[i][1])*0.5),2),round(path[i][2],2)]
            
            response=planner_srv._response_class()
            for ii in range(0,len(path)):
                response.path.append(path[ii][0])
                response.path.append(path[ii][1])
                response.path.append(path[ii][2])
                #Caracteristicas del la trayectoria para representarla en rviz
                marker = Marker()
                marker.header.frame_id = "/map"
                marker.type = marker.SPHERE
                marker.action = marker.ADD
                marker.scale.x = 0.4
                marker.scale.y = 0.4
                marker.scale.z = 0.4
                marker.color.a = 1.0
                marker.color.r = 1.0
                marker.color.g = 0.0
                marker.color.b = 0.0
                marker.pose.orientation.w = 1.0
                
                marker.pose.position.x=path[ii][0]
                marker.pose.position.y=path[ii][1]
                marker.pose.position.z=path[ii][2]
                markerArray.markers.append(marker)
                #Fin configuracion rviz
            id = 0
            for m in markerArray.markers:
                m.id = id
                id += 1
            
        
        self.final=time.time()
        print('PLANNER: PATHING COMPLETED, IT HAS TAKEN:' + str(round(self.final-self.inicio,2)) + ' SECONDS')
        
        return response

        
            
    def empezar(self):
        rospy.init_node('robot_planner', anonymous=True)
        s=rospy.Service('/del_uav/planner',planner_srv, self.compute_path)
        rate=rospy.Rate(0.2)
        while not rospy.is_shutdown():
            
            w.publish(markerArray)#Publico el markerArray para que rviz me represente la trayectoria
            rate.sleep()

        #rospy.spin()
        
if __name__ == '__main__':
   try:
        w=rospy.Publisher('/del_uav/path',MarkerArray,queue_size=10)
        bandera=0
        markerArray=MarkerArray()
        x = Planner() # crea el objeto del tipo Planner con todas las funciones
        x.empezar()
        exit()
        #rospy.spin()
   except rospy.ROSInterruptException:
      pass
