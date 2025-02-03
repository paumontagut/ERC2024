# UJI ROBOTICS - ERC2025

Hola! Este es el repositorio que contiene todo lo relacionado con el server del rover, todos los programas que se ejecutan en la [Jetson](https://drive.google.com/open?id=1fHbcS8U8frjhqGWFR6CukLB3bt-LdUYFgUYxeBtJ6mc&usp=drive_copy) y la conexión con los diferentes sensores y actuadores.

Este documento contiene un resumen de todos los pasos a seguir para poder hacer funcionar al rover desde 0, además de una explicación de qué hace cada parte del código. Para profundizar más en cualquier aspecto, podrás encontrar todos las memorias con los procesos en el siguiente documento: [Memoria Server](https://docs.google.com/document/d/1E_Q-_umBtsWQF4Fvs7V24_zVx3GvIHj7npbNcFVh6iw/edit?usp=sharing). También se tratará de enlazar en cada sección con la respectiva parte de la memoria.

## Índice

- [UJI ROBOTICS - ERC2025](#uji-robotics---erc2025)
    - [Índice](#índice)
    - [Requisitos](#requisitos)
    - [Ejecución básica - Mandar comandos](#ejecución-básica---mandar-comandos)
        - [GUI Principal](#gui-principal)
        - [Topics pre-definidos](#topics-pre-definidos)
        - [Comandos personalizados](#comandos-personalizados)
    - [Siguientes aspectos](#siguientes-aspectos)

## Requisitos

Por ahora la mayor parte de las pruebas se han hecho en la propia Jetson, puesto que es la que tiene todos los paquetes y dependencias necesarias instaladas. Tratar de compilar en un portátil propio puede conllevar a todo tipo de errores del que no nos haremos responsables por ahora, aunque la idea es organizar qué paquetes sí se pueden probar en un portátil y cuáles no.

Para poder mandar comandos al rover, deberemos estar conectados a la misma red y también tener los sources bien configurados en ambos dispositivos. En la jetson se habrá hecho todo automáticamente, solo quedaría en el portátil del usuario.

La conexión automática en la jetson se establecerá mediante `utilidades/net_connect.sh`.

Los sources y el ROS_DOMAIN_ID se establecerán mediante `utilidades/rover_sources.sh`, que también se deberá ejecutar en cada terminal que abra el usuario en su portátil.

- [ ] TODO: Hacer bien esto anterior, implementar antena, wifis posibles por prioridad...
- [ ] Mejorar rover_sources.sh, con comandos personalizados y rápidos, tuberias, mejores explicaciones, que aparezca al abrir terminal...

---

En caso de que la jetson no se conecte automáticamente a un internet, dé problemas, o simplemente se quiera programar de forma más avanzada o conectar a ella, hay varias opciones:

- Conectar la jetson a un monitor, teclado y raton, 
    - El usuario es `ujiroboticsteam` y la contraseña es `qwerty`.

- `ssh` básico
    - [ ] Falta revisar
    - Por el bot de telegram de sshx tendremos la IP, y con `ssh ujiroboticsteam@<ip>` y contraseña `qwerty` podremos también acceder.

- `sshx` avanzado colaborativo
    - Automáticamente cuando se abre la jetson se crea un link que se manda por el bot de telegram, con el que podemos acceder a terminales de la jetson colaborando con otros integrantes.
    - [ ] Link del grupo de telegram: ...


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

## Aspectos programación

### Programas de inicio automáticos

Son aquellos scripts que se encuentran en `ERC2024/startup_scripts/scripts`, que se ejecutarán siempre que se abra la jetson para configurar diversos aspectos. El script `ERC2024/startup_scripts/startup_scripts.sh` es el comando global que ejecutará el esto. Y en `ERC2024/startup_scripts/startup_scripts.log` se podrá debuggear la ejecución.

Entre los programas que se ejecutan están:

- jetson_sshx_bot_v4.sh
    - Explicado anteriormente

- start_supervisor.sh
    - Este es el nodo principal que se ejecuta siempre que encendemos el rover (ver más en [Nodo Supervisor - Google Docs](https://drive.google.com/open?id=1HD8huN-Qh6SthnQPIevO-fIPyWkLCJyxNCTryFgf2E0&usp=drive_copy)).

Y algunos comandos para arreglar errores o debuggear son:

    sudo nano /etc/systemd/system/rover-startup.service
    sudo systemctl daemon-reload

    sudo systemctl enable rover-startup.service
    sudo systemctl disable rover-startup.service

    sudo systemctl start rover-startup.service
    sudo systemctl stop rover-startup.service

    sudo systemctl status rover-startup.service
    journalctl -u rover-startup.service -f
    nano ER


---


[Programación](documentaci%C3%B3n/aspectos-programacion.md)
