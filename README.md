# UJI ROBOTICS - ERC2024

Ver también: [Normas de Código](./normas_código.md)

## Compilación

Estar en ~/ERC2024/test_ws

```rosdep install --from-paths src --ignore-src -r -y && \ 
 colcon build --packages-select custom_interfaces && \ 
 colcon build --packages-select dynamixel_sdk && \ 
 source install/setup.bash && \ 
 colcon build --symlink-install --packages-select bringup && \ 
 colcon build
```

## Paquetes

**bringup**

- Contiene todos los nodos necesarios para el funcionamiento del robot.
- `ros2 launch bringup robot_brinup.launch.py`

**camera_management**

- launches extra que se llaman desde bringup.
- De momento solo están las logitech.
- `ros2 launch camera_management logitech_cameras.launch.py`

**custom_interfaces**

- Mensajes, servicios y acciones personalizados.

**movement**

- Control del movimiento.
- `ros2 run movement keyboard_teleop`