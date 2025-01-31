# UJI ROBOTICS - ERC2025

Hola! Este es el repositorio que contiene todo lo relacionado con el server del rover, todos los programas que se ejecutan en la [Jetson](https://drive.google.com/open?id=1fHbcS8U8frjhqGWFR6CukLB3bt-LdUYFgUYxeBtJ6mc&usp=drive_copy) y la conexión con los diferentes sensores y actuadores.

Este documento contiene un resumen de todos los pasos a seguir para poder hacer funcionar al rover desde 0, además de una explicación de qué hace cada parte del código. Para profundizar más en cualquier aspecto, podrás encontrar todos las memorias con los procesos en el siguiente documento: [Memoria Server](https://docs.google.com/document/d/1E_Q-_umBtsWQF4Fvs7V24_zVx3GvIHj7npbNcFVh6iw/edit?usp=sharing). También se tratará de enlazar en cada sección con la respectiva parte de la memoria.

## Índice

- [UJI ROBOTICS - ERC2025](#uji-robotics---erc2025)
    - [Índice](#índice)
    - [Pre-Conocimentos, Conexión a la misma red](#pre-conocimentos-conexión-a-la-misma-red)
    - [Ejecución básica - Mandar comandos](#ejecución-básica---mandar-comandos)
        - [GUI Principal](#gui-principal)
        - [Topics pre-definidos](#topics-pre-definidos)
        - [Comandos personalizados](#comandos-personalizados)
    - [Siguientes aspectos](#siguientes-aspectos)

## Pre-Conocimentos, Conexión a la misma red

Por ahora la mayor parte de las pruebas se han hecho en la propia Jetson, puesto que es la que tiene todos los paquetes y dependencias necesarias instaladas. 

Tratar de compilar en un portátil propio puede conllevar a todo tipo de errores del que no nos haremos responsables por ahora, aunque la idea es organizar qué paquetes sí se pueden probar en un portátil y cuáles no.

Para poder mandar comandos al rover, deberemos estar conectados a la misma red y también tener en ambos lo siguiente, que en principio se debería hacer solo al abrir la jetson, pero en el portátil de cada uno si que es importante:

`unset ROS_DOMAIN_ID`
`export ROS_LOCALHOST_ONLY=0`

- [ ] TODO: Antena, rosdomain, ips... Arreglar y automatizar.
- [ ] Función personalizada (bashrc) que todo el mundo haga en su portatil, con sources, rosdomain, tuberias para comandos personalizados...
- [ ] Mirar ssh

## Ejecución básica - Mandar comandos

Para mover el rover o ejecutar programas encenderemos el rover y automáticamente se abrirá el Nodo Supervisor que se encargará de lanzar cualquier programa que le mandemos.

Una vez ya estemos conectados a la misma red que el rover y este esté encendido, ya podremos moverlo y ejecutar programas. Para ello tenemos varias opciones:

### GUI Principal
- Usar la GUI principal creada específicamente para el ERC2025 (pedir a Guillem, Adrià, etc.)
- [ ] TODO: Link a releases github, con fichero ubuntu, windows, android... y tener aqui para cceso facil
- Esta se encarga de automáticamente mandar todos los topics necesarios, visualizar las cámaras [...]



### Topics pre-definidos

- Acceder a topics predefinidos desde cualquier terminal
    - Para ello se podrá mandar "true/false" en los diferentes topics ya creados para ejecutar o cancelar cualquier programa. El comando general es el siguiente:
        - `ros2 topic pub -1 {topic} std_msgs/Bool "data: true"`
        - `ros2 topic pub -1 {topic} std_msgs/Bool "data: false"`
    - Donde los topics posibles son los siguientes:
        - `/gui/bringup`
            - ¿qué hace? mira [Rover Bringup](#rover-bringup)
        - `/gui/shutdown`
            - Apaga el rover.
        - `/gui/ruedas`
        - `/gui/all_cameras`
        - ...
        - `/gui/logitech_cameras`
        - `/gui/logitech_cameras_1`
        - `/gui/logitech_cameras_2`
        - `/gui/realsense_camera`
        - `/gui/unitree_lidar`
        - `/gui/zed2_motors`
        - [...] TODO: Revisar
    - Todos están en `/home/ujiroboticsteam/ERC2024/erc_ws/src/rover_supervisor/rover_supervisor/config/topics_programas.py`... explicar mejor

### Comandos personalizados

Aún está en fase de pruebas, pero la opción principal es mandar comandos individuales y que se ejecuten en la jetson.

Para ello se ejecutará el comando `ros2 topic pub -1 /gui/terminal_input std_msgs/String "data: '{comando}'"`, donde `{comando}` es lo que queremos ejecutar.

Para ver el output de los comandos, podremos hacerlo en otra terminal con `ros2 topic echo /gui/terminal_output`.

- [ ] TODO: Revisar mejor. simplificar comando con echo y cosas así tuberias... royo `ls > rover`

## Siguientes aspectos

[Programación](documentaci%C3%B3n/aspectos-programacion.md)
