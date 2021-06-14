# delivery_uav
 Proyecto para Robotica Avanzada (GITI) / Ampliación de Robótica (GIERM)
 
## Instalación
Versión de ubuntu 16.04 / ROS kinetic
 * Instalar ROS: http://wiki.ros.org/es/ROS/Installation
 * Instalar UAL: https://github.com/grvcTeam/grvc-ual/tree/master/uav_abstraction_layer
 * Instalar dependencias necesarias:
   - numpy
   - threading
   - ros_numpy
   - joint state publisher gui
   - find object 2d
   - rtabmap y rtabmap_ros
   - python_pcl y pcl_ros
   - open3d
   - open3d_ros_helper
   - octomap y octomap_server (opcional para representación de mapas 3D)
   - map_server
 
## Manual de usuario
Para manejar el sistema, el único servicio que el usuario ha de conocer es /del_uav/user_interface. Este servicio acepta distintos comandos en forma de palabras clave
que permiten al usuario controlar su comportamiento. Estos comandos son:
* 'start': Hace que el UAV se eleve a una altura considerada segura (3 metros). Inicializa el sistema y desbloquea el resto de comandos.
* 'auto': Además de la palabra clave, esta llamada se debe acompañar de un punto válido del mapa, que este a más de metro y medio de cualquier pared. Hace que el UAV se desplace a dicho punto siguiendo una trayectoria segura.
* 'drop': Abre la garra una vez el UAV esta estable, dejando caer el paquete.
* 'gohome': Hace que el UAV vuelva a su posición inicial, siguiendo una ruta segura. Una vez llega allí, aterriza en dicho punto y reinicia el sistema.
* 'idle': Aborta el movimiento actual. Se debe realizar desde otra terminal y anulará el movimiento una vez se alcance el siguiente waypoint.
