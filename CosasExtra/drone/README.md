# UJI ROBOTICS - ERC2024 - Drone parrot bebop 2


### Requerimientos
* Ubuntu 20.02
* ROS Noetic (En futuras actualizaciones posiblemente disponible en ROS2)

 ### Instalación de paquetes y dependencias relativas
 Se crea un workespace bebop_ws donde almacenar los paquetes y programas del drone.
```
mkdir -p bebop_ws/src && cd bebop_ws/src

sudo apt install build-essential python3-rosdep python3-catkin-tools
sudo apt install libusb-dev python3-osrf-pycommon libspnav-dev libbluetooth-dev libcwiid-dev libgoogle-glog-dev
sudo apt install ros-noetic-mavros ros-noetic-octomap-ros 

git clone https://github.com/ethz-asl/mav_comm
git clone -b noetic https://github.com/simonernst/iROS_drone
git clone https://github.com/ros-drivers/joystick_drivers

source /opt/ros/noetic/setup.bash
cd ..
catkin build

```
#### Estos paquetes estan validados para la simulación del drone, aún no esta probado con el drone fïsico.
