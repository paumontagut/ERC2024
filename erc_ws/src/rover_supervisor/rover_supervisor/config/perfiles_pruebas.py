PERFILES_PRUEBAS = {
    "gui_teleop": {
        "/gui/logitech_cameras": {
            "enable_camara1": "true",
            "enable_camara2": "true",
            "path_camara1": "/dev/video0",
            "path_camara2": "/dev/video2"
        },
        
        "/gui/unitree_lidar": {}
    },
    
    "auto_nav": {
        "/gui/logitech_cameras": {
            "enable_camara1": "true",
            "enable_camara2": "false",
            "path_camara1": "/dev/video0"
        },
        "/gui/realsense_camera": {
            "camera_depth": "true"
        }
    },
}
