# UJI ROBOTICS - ERC2024

Ver también: [Normas de Código](./documentación/normas_código.md)

## Compilación

`cd ~/ERC2024/erc_ws`
`colcon build`
`source install/setup.bash`

Debería ser suficiente con lo anterior.
Detecta automáticamente las dependencias y el orden para hacerlo.
Si diera error, ejecutar lo siguiente:

`rosdep install --from-paths src --ignore-src -r -y && colcon build --packages-select custom_interfaces && colcon build --packages-select dynamixel_sdk && source install/setup.bash && colcon build --symlink-install --packages-select rover_bringup && colcon build`

## Rover Supervisor

- Programa a ejecutar siempre, cuando se enciende el robot.
- `ros2 run rover_supervisor supervisor_node`
- Dentro también está el "supervisor_node_persistent_terminal", que es un testeo para poder clonar el funcionamiento de la terminal para los de unity.

## Rover Bringup

- Contiene todos los launches necesarios para el funcionamiento del robot. 
- Tiene uno principal que lanza todos los demás:
    - `ros2 launch rover_bringup rover_bringup.launch.py`
- Y luego los sub-launches, que cada uno ejecuta un paquete de [Sensores Actuadores](#sensores-actuadores):
  - `ros2 launch rover_bringup logitech_cameras.launch.py`
  - `ros2 launch rover_bringup realsense_camera.launch.py`
  - `ros2 launch rover_bringup ruedas.launch.py`
  - `ros2 launch rover_bringup unitree_lidar.launch.py`

## Sensores Actuadores

En esta carpeta se encuentran todos los paquetes necesarios, que son llamados desde el [Rover Bringup](#rover-bringup)

## Drivers

En esta carpeta se encuentran todos los paquetes que no se ejecutan directamente, pero que hacen falta para que otros paquetes funcionen correctamente.

## Programas

Aquí se irán poniendo los diferentes paquetes de las pruebas.
Por ahora solo está "movement", que aún hay que testearlo bien.

## Argumentos

- Podemos ver argumentos que reciben los sub-launches (por si queremos hacer pruebas) con por ejemplo:
- `ros2 launch rover_bringup logitech_cameras.launch.py --show-args`

- Además, al launch principal también la idea sería poder pasarle estos mismos argumentos y que se pasara a su vez a los sub-launches, haciendolo personalizable al 100%:
- Por ahora no se ha implementado esto, se vería con:
- `ros2 launch rover_bringup rover_bringup.launch.py --show-args`