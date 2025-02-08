topics_programas = {

# ================================
# ======== BRINGUP GLOBAL ========
# ================================
# Activa todos los sensores y actuadores
'/gui/rover_bringup': {     # TODO: Revisar bien.
    'command': 'launch',
    'package': 'rover_bringup',
    'executable_or_file': 'rover_bringup.launch.py',
    'arguments': {},
},

'/gui/file_receptor': {
    'command': 'exec',
    'package': None,
    'executable_or_file': 'python3 file_receptor.py',
    'arguments': {},
},

'/gui/shutdown': {
    'command': 'exec',
    'package': None,
    'executable_or_file': 'sudo halt -p',
    'arguments': {},
},

# ========= CAMARAS =======

'/gui/logitech_camara_derecha': {
    'command': 'launch',
    'package': 'rover_bringup',
    'executable_or_file': 'logitech_cameras.launch.py',
    'arguments': {
        'enable_camara_derecha': 'true',
        'enable_camara_izquierda': 'false',
    },
},

'/gui/logitech_camara_izquierda': {
    'command': 'launch',
    'package': 'rover_bringup',
    'executable_or_file': 'logitech_cameras.launch.py',
    'arguments': {
        'enable_camara_derecha': 'false',
        'enable_camara_izquierda': 'true',
    },
},

'/gui/realsense_camera': {
    'command': 'launch',
    'package': 'realsense2_camera',
    'executable_or_file': 'rs_launch.py',
    'arguments': {
        'align_depth.enable': 'false',
        'pointcloud.enable': 'false',
        'camera_depth': 'true'
    },
},

# '/gui/siyi_camera': {
#     # Pendiente de implementar
# },

# '/gui/zed2_camera': {   # TODO: implementar
# },


# ========= LIDARS =========

'/gui/unitree_lidar': {
    'command': 'launch',
    'package': 'unitree_lidar_ros2',
    'executable_or_file': 'unitree_lidar.launch.py',
    'arguments': {
        'serial_port': '/dev/unitree_lidar'
    },
},

'/gui/lidar_2d': {
    'command': 'launch',
    'package': 'rover_bringup',
    'executable_or_file': 'lidar2d.launch.py',
    'arguments': {
        'serial_port': '/dev/lidar2d'
    },
},

# ========= MOTORS =========

# '/gui/all_motors': {
#     'command': 'launch',
#     'package': 'rover_bringup',
#     'executable_or_file': 'all_motors.launch.py',
#     'arguments': {},
# },

'/gui/ruedas': {
    'command': 'run',
    'package': 'rover_motor_controller',
    'executable_or_file': 'motor_vel_controller',
    'arguments': {'/dev/ruedas'},
},

'/gui/zed2_motors': {
    'command': 'launch',
    'package': 'rover_bringup',
    'executable_or_file': 'zed2_motors.launch.py',
    'arguments': {},
},

# ========= OTROS =========
# '/gui/semaforo': {
#     # Pendiente de implementar
# },

}
