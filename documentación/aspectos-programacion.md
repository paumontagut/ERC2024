
## Aspectos avanzados y programación

### Compilación

- [ ] TODO: Agregar explicación compilación en Aspectos básicos

`cd ~/ERC2024/erc_ws`
`colcon build`
`source install/setup.bash`

Debería ser suficiente con lo anterior.
Detecta automáticamente las dependencias y el orden para hacerlo.
Si diera error, ejecutar lo siguiente:

`rosdep install --from-paths src --ignore-src -r -y && colcon build --packages-select custom_interfaces && colcon build --packages-select dynamixel_sdk && source install/setup.bash && colcon build --symlink-install --packages-select rover_bringup && colcon build`


### Rover Bringup

- Contiene todos los launches necesarios para el funcionamiento del robot. 
- Tiene uno principal que lanza todos los demás:
    - `ros2 launch rover_bringup rover_bringup.launch.py`
- Y luego los sub-launches, que cada uno ejecuta un paquete de [Sensores Actuadores](#sensores-actuadores):
  - `ros2 launch rover_bringup logitech_cameras.launch.py`
  - `ros2 launch rover_bringup realsense_camera.launch.py`
  - `ros2 launch rover_bringup ruedas.launch.py`
  - `ros2 launch rover_bringup unitree_lidar.launch.py`

Comandos:

- `ros2 topic pub -1 {topic} std_msgs/Bool "data: true"`
    - `/gui/ruedas`
    - `/gui/logitech_cameras_1`
    - `/gui/logitech_cameras_2`
    - `/gui/realsense_camera`
    - `/gui/unitree_lidar`
    - `/gui/zed2_motors`
    - `/gui/shutdown`
    - El resto falta revisar/implementar: zed2_camera, all_cameras, siyi, lidar2d, all_motors, semaforo...
- `ros2 topic echo /gui/terminal_output`
- `ros2 topic pub -1 /program/update_arguments std_msgs/String "data: '{topic},{key},{argument}'"`
    - topic: `/gui/ruedas`
        - key: `device`, value: `/dev/ttyUSB0`
        - key: `device`, value: `/dev/ttyUSB1`
    - topic: `/gui/unitree_lidar`
        - key: `serial_port`, value: `/dev/ttyUSB0`
        - key: `serial_port`, value: `/dev/ttyUSB1`

### Sensores Actuadores

En esta carpeta se encuentran todos los paquetes necesarios, que son llamados desde el [Rover Bringup](#rover-bringup)

### Drivers

En esta carpeta se encuentran todos los paquetes que no se ejecutan directamente, pero que hacen falta para que otros paquetes funcionen correctamente.

### Programas

Aquí se irán poniendo los diferentes paquetes de las pruebas.
Por ahora solo está "movement", que aún hay que testearlo bien.

### Argumentos

- Podemos ver argumentos que reciben los sub-launches (por si queremos hacer pruebas) con por ejemplo:
- `ros2 launch rover_bringup logitech_cameras.launch.py --show-args`

- Además, al launch principal también la idea sería poder pasarle estos mismos argumentos y que se pasara a su vez a los sub-launches, haciendolo personalizable al 100%:
- Por ahora no se ha implementado esto, se vería con:
- `ros2 launch rover_bringup rover_bringup.launch.py --show-args`