# delivery_uav
 Proyecto para Robotica Avanzada (GITI) / Ampliación de Robótica (GIERM)
 
## Instalación
Versión de ubuntu 16.04 / ROS kinetic
 * Instalar ROS: http://wiki.ros.org/es/ROS/Installation
 * Instalar UAL: https://github.com/grvcTeam/grvc-ual/wiki/How-to-build-and-install-grvc-ual
   - Seguir setup detallado
   - Instalar px4 v1.7.3 anteriormente: https://github.com/grvcTeam/grvc-ual/wiki/Setup-instructions:--PX4-SITL-(v1.7.3)-(OLD)
 * Actualizar pip para Python 2.7:
   - wget https://bootstrap.pypa.io/pip/2.7/get-pip.py
   - python get-pip.py
 * Instalar versión específica de gcc:
   - sudo add-apt-repository ppa:ubuntu-toolchain-r/test
   - sudo apt-get update
   - sudo apt-get install gcc-4.9
   - sudo apt-get upgrade libstdc++6

 * Instalar dependencias necesarias:
   - numpy (pip install numpy)
   - threading (instalado con Python)
   - ros_numpy (sudo apt install ros-kinetic-ros-numpy)
   - joint state publisher (sudo apt install ros-kinetic-joint-state-publisher-gui)
   - find object 2d (se instala con rtabmap_ros)
   - rtabmap y rtabmap_ros (sudo apt install ros-kinetic-rtabmap-ros)
   - python_pcl (pip install python_pcl)
   - open3d (pip install open3d=0.9.0)
   - open3d_ros_helper (https://pypi.org/project/open3d-ros-helper)
   - octomap y octomap_server (opcional para representación de mapas 3D) (paquetes de ROS)
   - map_server (sudo apt install ros-kinetic-map-server)

Los archivos icp_node son los que usan las librerías de open3d y python_pcl, a pesar de que en la versión final no se usan.
En caso de fallos al ejecutar KF_node_T.py relacionados con librerías:
   - Buscar la línea 193 de código y cambiarla por "self.T = ros_numpy.numpify(data.transform)"


Se proporciona una copia de Firmware y de grvc-ual en caso de que halla fallos al seguir la instalación

 * Clonar paquete rtabmap_ros al workspace en caso de fallo: http://wiki.ros.org/rtabmap_ros
 


