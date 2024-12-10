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

# ======= SENSORES =======

'/gui/logitech_cameras': {
    'command': 'launch',
    'package': 'rover_bringup',
    'executable_or_file': 'logitech_cameras.launch.py',
    'arguments': {
        'enable_camara1': 'true',
        'enable_camara2': 'true',
        'path_camara1': '/dev/video0',
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

'/gui/unitree_lidar': {
    'command': 'launch',
    'package': 'unitree_lidar_ros2',
    'executable_or_file': 'launch.py',
    'arguments': {
        'serial_port': '/dev/ttyUSB0'
    },
},

'/gui/siyi_camera': {
    # Pendiente de implementar
},


# ======= ACTUADORES =======

'/gui/ruedas': {
    'command': 'launch',
    'package': 'rover_bringup',
    'executable_or_file': 'motor_controller',
    'arguments': {},
},

'/gui/semaforo': {
    # Pendiente de implementar
},

}
