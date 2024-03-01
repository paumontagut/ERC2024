# UJI ROBOTICS - ERC2024 - Drone parrot bebop 2 - Simulación


### Lanzamiento modo simulación

```
source /opt/ros/noetic/setup.bash
cd ~/bebop_ws 
catkin build
source devel/setup.bash
roslaunch rotors_gazebo mav_velocity_control_with_fake_driver.launch

```


### Mostrar video de la cámara 
```
source /opt/ros/noetic/setup.bash
rqt_image_view

```
#### Selecionar /bebop2/camera_base/image_raw/compressed



### Envío de comandos desde terminal
```
source /opt/ros/noetic/setup.bash
rostopic pub --once /bebop/takeoff std_msgs/Empty
rostopic pub --once /bebop/cmd_vel geometry_msgs/Twist '{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.2}}’

```
### Programa
Prueba de simulación de parrot en rospy 
```
cd ~/GitHub/ERC2024/test_ws/src/drone
```
#### Depende de donde has clonado el repositorio


```
source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash
rosrun simulacion prueba_simulacion.py
```
#### Programa sencillo. Despega, sube cinco metros, realiza una fotogafia (que se guarda en la carpeta drone/images) y aterriza.

