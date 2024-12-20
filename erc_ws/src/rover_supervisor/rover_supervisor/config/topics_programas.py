topics_programas = {

# ================================
# ======== BRINGUP GLOBAL ========
# ================================
# Activa todos los sensores y actuadores
'/gui/rover_bringup': {
    'command': 'launch',
    'package': 'rover_bringup',
    'executable_or_file': 'rover_bringup.launch.py',
    'arguments': {},
},

'/gui/shutdown': {
    'command': 'exec',
    'package': None,
    'executable_or_file': 'sudo apt update',
    'arguments': {},
},

# ========= CAMARAS =======

# '/gui/all_cameras': {      # TODO: Implementar
#     'command': 'launch',
#     'package': 'rover_bringup',
#     'executable_or_file': 'all_cameras.launch.py',
#     'arguments': {},
# },

# '/gui/logitech_cameras': {
#     'command': 'launch',
#     'package': 'rover_bringup',
#     'executable_or_file': 'logitech_cameras.launch.py',
#     'arguments': {
#         'enable_camara1': 'true',
#         'enable_camara2': 'true',
#         'path_camara1': '/dev/video0',
#         'path_camara2': '/dev/video2'
#     },
# },

'/gui/logitech_cameras_1': {
    'command': 'launch',
    'package': 'rover_bringup',
    'executable_or_file': 'logitech_cameras.launch.py',
    'arguments': {
        'enable_camara1': 'true',
        'enable_camara2': 'false',
        'path_camara1': '/dev/video0'
    },
},

'/gui/logitech_cameras_2': {
    'command': 'launch',
    'package': 'rover_bringup',
    'executable_or_file': 'logitech_cameras.launch.py',
    'arguments': {
        'enable_camara1': 'false',
        'enable_camara2': 'true',
        'path_camara2': '/dev/video2'
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
        'serial_port': '/dev/ttyUSB1'
    },
},

# '/gui/lidar_2d': {
#     'command': 'launch',
#     'package': 'rover_bringup',
#     'executable_or_file': 'lidar_2d.launch.py',
#     'arguments': {},
# },

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
    'arguments': {
        'device': '/dev/ttyUSB0',
    },
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
