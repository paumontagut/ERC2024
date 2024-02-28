# UJI ROBOTICS - ERC2024 - Drone parrot bebop 2


### Lanzamiento modo simulación

```
source /opt/ros/noetic/setup.bash
cd ~/bebop_ws 
catkin build
source devel/setup.bash
roslaunch rotors_gazebo mav_velocity_control_with_fake_driver.launch
```
#### Estos paquetes estan validados para la simulación del drone, aún no esta probado con el drone fïsico.
