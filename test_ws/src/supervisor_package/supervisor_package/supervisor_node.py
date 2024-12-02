import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import subprocess
import os
import signal

class LaunchManagerNode(Node):
    def __init__(self):
        super().__init__('supervisor_node')
        self.get_logger().info('Supervisor node started. Send any of the following topics:')

        # ========= LAUNCHES POSIBLES =========
        self.launches = {
            '/execute/bringup': {
                'package': 'bringup',
                'launch_file': 'robot_bringup.launch.py',
                'process': None     # Se guarda el proceso iniciado para poder cerrarlo después.
            },
            
            '/execute/camera1_logitech': {
                'package': 'camera_management',
                'launch_file': 'logitech_test1.launch.py',
                'process': None
            },
            
            '/execute/camera2_logitech': {
                'package': 'camera_management',
                'launch_file': 'logitech_test2.launch.py',
                'process': None
            },
            
            '/execute/both_cams_logitech': {
                'package': 'camera_management',
                'launch_file': 'logitech_cameras.launch.py',
                'process': None
            },
            
            # ............................
        }
        
        for topic in self.launches.keys():
            self.get_logger().info(f' - {topic}')


        # ==== Crear suscripciones para todos los topics de arriba ====
        for topic in self.launches.keys():
            self.create_subscription(
                Bool,
                topic,
                lambda msg, t=topic: self.launch_callback(msg, t),
                10
            )

    def launch_callback(self, msg, topic_name):
        # Función que se ejecuta cada vez que se recibe cualquier topic de los iniciados. Y se iniciará o cerrará el launch correspondiente.
        self.get_logger().info(f'Received message on {topic_name}: {msg.data}')

        launch_info = self.launches[topic_name]

        if msg.data:
            # No iniciado -> iniciar
            if launch_info['process'] is None:
                self.get_logger().info(f'Starting launch file for {topic_name}...')
                self.start_launch(topic_name)
            # Iniciado -> no hacer nada
            else:
                self.get_logger().info(f'Launch file for {topic_name} already running.')
        else:
            # Iniciado -> cerrar
            if launch_info['process'] is not None:
                self.get_logger().info(f'Stopping launch file for {topic_name}...')
                self.stop_launch(topic_name)
            # No iniciado -> no hacer nada
            else:
                self.get_logger().info(f'No launch file for {topic_name} is running.')

    def start_launch(self, topic_name):
        # Buscar path del launch, y crear el comando que se ejecutará
        launch_info = self.launches[topic_name]
        package = launch_info['package']
        launch_file_name = launch_info['launch_file']
        cmd = [
            'ros2', 'launch', package, launch_file_name
        ]

        # Inicializar el proceso, guardar un id
        # Esto ejecuta un proceso en paralelo, sin detener el programa principal.
        # preexec_fn=os.setsid es para darle independencia completa de la terminal, y luego ejecutar el comando principal.
        # - Esto ayuda a que si se cierra el nodo principal, no se cierre el proceso lanzado.
        process = subprocess.Popen(cmd, preexec_fn=os.setsid)
        launch_info['process'] = process


    def stop_launch(self, topic_name):
        launch_info = self.launches[topic_name]
        process = launch_info['process']

        if process:
            self.get_logger().info(f'Terminating process group for {topic_name}...')
            try:
                # Dado el id que guardamos, lo busca y lo mata (con SIGTERM, es decir, de forma controlada, como si hicieramos Ctrl+C)
                os.killpg(os.getpgid(process.pid), signal.SIGTERM)
                process.wait(timeout=5)
                self.get_logger().info(f'Process group for {topic_name} terminated.')
            except subprocess.TimeoutExpired:
                self.get_logger().warning(f'Process group for {topic_name} did not terminate in time. Killing...')
                # Y sí no es capaz por las buenas, se hace por las malas (SIGKILL, es decir, MATARRRRRRR, SANGREEEEEEEE)
                os.killpg(os.getpgid(process.pid), signal.SIGKILL)
            except Exception as e:
                self.get_logger().error(f'Error terminating process group for {topic_name}: {e}')
            launch_info['process'] = None


    def destroy_node(self):
        # Al cerrar el nodo supervisor nos aseguramos de cerrar todos los procesos lanzados para que no se queden por ahí sueltos sin rumbo.
        for topic_name in self.launches.keys():
            process = self.launches[topic_name]['process']
            if process:
                self.stop_launch(topic_name)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = LaunchManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
