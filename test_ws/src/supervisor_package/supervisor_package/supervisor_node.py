import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from std_msgs.msg import String
import subprocess
import os
import signal
from threading import Lock, Thread

class SupervisorNode(Node):
    def __init__(self):
        super().__init__('supervisor_node')

        self.get_logger().info('Supervisor node started. Listening for program management and command execution topics.')

        # Launchable programs: both `ros2 launch` and `ros2 run`
        # Format: 
        #   - 'command': either 'launch' or 'run'
        #   - 'package': the ROS 2 package
        #   - 'executable_or_file': the executable (for `run`) or the launch file name (for `launch`)
        #   - 'arguments': a dictionary of optional arguments to pass to the program
        self.programs = {
            '/gui/camera_management': {
                'command': 'launch',
                'package': 'camera_management',
                'executable_or_file': 'logitech_cameras.launch.py',
                'arguments': {
                    'enable_camara1': 'true',
                    'enable_camara2': 'true',
                    'path_camara1': '/dev/video0',
                    'path_camara2': '/dev/video2'
                },
                'process': None
            },
            # '/program/turtlesim_node': {
            #     'command': 'run',
            #     'package': 'turtlesim',
            #     'executable_or_file': 'turtlesim_node',
            #     'arguments': {
            #         'background_r': '255',
            #         'background_g': '255',
            #         'background_b': '255'
            #     },
            #     'process': None
            # },
            # Add more programs here as needed
        }

        # Lock for thread safety
        self.process_lock = Lock()

        # Create subscriptions for each program
        for topic in self.programs.keys():
            self.create_subscription(
                Bool,
                topic,
                lambda msg, t=topic: self.program_callback(msg, t),
                10
            )

        # Subscription for updating arguments dynamically
        self.create_subscription(
            String,
            '/program/update_arguments',
            self.update_arguments_callback,
            10
        )

        # ======== Remote Command Execution ========
        # Subscription for executing arbitrary terminal commands
        self.create_subscription(
            String,
            '/gui/terminal_input',
            self.execute_command_callback,
            10
        )

        # Publisher for command execution results
        self.command_output_publisher = self.create_publisher(
            String,
            '/gui/terminal_output',
            10
        )

    def program_callback(self, msg, topic_name):
        """Función que se ejecuta cada vez que se recibe cualquier topic de los iniciados. Y se iniciará o cerrará el programa correspondiente"""
        with self.process_lock:
            program = self.programs[topic_name]
            self.get_logger().info(f'Received {msg.data} for {topic_name}')
            if msg.data:  # True -> Start the program
                if program['process'] is None:
                    self.get_logger().info(f'Starting program: {topic_name}')
                    self.start_program(topic_name)
                else:
                    self.get_logger().info(f'Program {topic_name} is already running.')
            else:  # False -> Stop the program
                if program['process'] is not None:
                    self.get_logger().info(f'Stopping program: {topic_name}')
                    self.stop_program(topic_name)
                else:
                    self.get_logger().info(f'Program {topic_name} is not running.')

    def update_arguments_callback(self, msg):
        """Update arguments for a program dynamically."""
        try:
            # Message format: topic_name,arg_key,arg_value
            topic_name, arg_key, arg_value = msg.data.split(',')
            if topic_name in self.programs:
                with self.process_lock:
                    program = self.programs[topic_name]
                    if arg_key in program['arguments']:
                        self.get_logger().info(f'Updating argument {arg_key} for {topic_name} to {arg_value}')
                        program['arguments'][arg_key] = arg_value
                        if program['process'] is not None:
                            self.get_logger().info(f'Restarting program {topic_name} with updated arguments...')
                            self.stop_program(topic_name)
                            self.start_program(topic_name)
                    else:
                        self.get_logger().warning(f'Argument {arg_key} not found for {topic_name}')
            else:
                self.get_logger().warning(f'Topic {topic_name} not found in programs.')
        except ValueError:
            self.get_logger().error('Invalid format for update_arguments message. Expected "topic_name,arg_key,arg_value".')

    def start_program(self, topic_name):
        """Start a program (launch or run) based on its configuration."""
        program = self.programs[topic_name]
        cmd = ['ros2', program['command'], program['package'], program['executable_or_file']]

        # Add arguments
        if program['arguments']:
            for arg_key, arg_value in program['arguments'].items():
                cmd.append(f'{arg_key}:={arg_value}')

        self.get_logger().info(f'Executing command: {" ".join(cmd)}')

        # Start the process
        process = subprocess.Popen(
            cmd,
            preexec_fn=os.setsid,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE
        )
        program['process'] = process
        self.get_logger().info(f'Process for {topic_name} started with PID: {process.pid}')

    def stop_program(self, topic_name):
        """Stop the currently running process for the program."""
        program = self.programs[topic_name]
        process = program['process']

        if process:
            self.get_logger().info(f'Stopping process for {topic_name}...')
            try:
                os.killpg(os.getpgid(process.pid), signal.SIGTERM)
                process.wait(timeout=5)
                self.get_logger().info(f'Process for {topic_name} terminated.')
            except subprocess.TimeoutExpired:
                # Y sí no es capaz por las buenas, se hace por las malas (SIGKILL, es decir, MATARRRRRRR, SANGREEEEEEEE)
                self.get_logger().warning(f'Process for {topic_name} did not terminate in time. Killing...')
                os.killpg(os.getpgid(process.pid), signal.SIGKILL)
                self.get_logger().info(f'Process for {topic_name} killed.')
            except Exception as e:
                self.get_logger().error(f'Error stopping process for {topic_name}: {e}')
            program['process'] = None

    # ======== Remote Command Execution Methods ========

    def execute_command_callback(self, msg):
        """Callback to execute a terminal command received via the topic."""
        command = msg.data.strip()
        if not command:
            self.get_logger().error('Received empty command. Ignoring.')
            return

        self.get_logger().info(f'Executing remote command: {command}')
        # Execute the command in a separate thread to avoid blocking
        Thread(target=self.run_command, args=(command,)).start()

    def run_command(self, command):
        """Run the given terminal command and publish its output or error."""
        try:
            # Execute the command
            result = subprocess.run(
                command,
                shell=True,
                text=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE
            )

            # Prepare the result message
            output_message = String()
            if result.returncode == 0:
                output_message.data = f'[SUCCESS]: {result.stdout.strip()}'
                self.get_logger().info(f'Command output: {result.stdout.strip()}')
            else:
                output_message.data = f'[ERROR]: {result.stderr.strip()}'
                self.get_logger().error(f'Command error: {result.stderr.strip()}')

            # Publish the result
            self.command_output_publisher.publish(output_message)

        except Exception as e:
            self.get_logger().error(f'Error executing command: {e}')
            error_message = String()
            error_message.data = f'[EXCEPTION]: {str(e)}'
            self.command_output_publisher.publish(error_message)

    def destroy_node(self):
        """Ensure all processes are stopped before shutting down."""
        self.get_logger().info('Shutting down supervisor node...')
        with self.process_lock:
            for topic_name in self.programs.keys():
                if self.programs[topic_name]['process']:
                    self.stop_program(topic_name)
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = SupervisorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt received. Shutting down.')
    finally:
        node.destroy_node()
        rclpy.shutdown()
