import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from std_msgs.msg import String
import subprocess
import os
import signal
from threading import Lock, Thread
from rover_supervisor.config.topics_programas import topics_programas

PASSWORD = "qwerty"  # TODO: Replace with actual password

class SupervisorNode(Node):
    def __init__(self):
        super().__init__('supervisor_node')
        self.command_output_publisher = self.create_publisher(String, '/gui/terminal_output', 10)
        
        # Log supervisor start
        self.log_and_publish('Supervisor node started. Listening for program management and command execution topics.')
        
        # ============== Bringups =================
        self.topics_programas = topics_programas
        for program in self.topics_programas.values():
            program.setdefault('process', None)

        self.process_lock = Lock()  # Safety lock

        for topic in self.topics_programas.keys():
            self.create_subscription(
                Bool,
                topic,
                lambda msg, t=topic: self.program_callback(msg, t),
                10
            )

        # ============== Updating arguments ==============
        self.create_subscription(
            String,
            '/program/update_arguments',
            self.update_arguments_callback,
            10
        )

        # ======== Remote Commands input/output ========
        self.create_subscription(
            String,
            '/gui/terminal_input',
            self.execute_command_callback,
            10
        )

        self.command_output_publisher = self.create_publisher(
            String,
            '/gui/terminal_output',
            10
        )

    def log_and_publish(self, message, level='info'):
        """Log a message and publish it to /gui/terminal_output."""
        # Log message with the appropriate severity
        if level == 'info':
            self.get_logger().info(message)
        elif level == 'warn':
            self.get_logger().warning(message)
        elif level == 'error':
            self.get_logger().error(message)
        else:
            self.get_logger().info(message)  # Default to info

        # Publish the same message to /gui/terminal_output
        output_message = String()
        output_message.data = message
        self.command_output_publisher.publish(output_message)



    def program_callback(self, msg, topic_name):
        """Función que se ejecuta cada vez que se recibe cualquier topic de los iniciados. Y se iniciará o cerrará el programa correspondiente"""
        with self.process_lock:
            program = self.topics_programas[topic_name]
            self.log_and_publish(f'Received {msg.data} for {topic_name}')
            # self.get_logger().info(f'Received {msg.data} for {topic_name}')
            
            if msg.data:  # True -> Start the program
                if program['process'] is None:
                    self.log_and_publish(f'Starting program: {topic_name}')
                    self.start_program(topic_name)
                else:
                    self.log_and_publish(f'Program {topic_name} is already running.')
            else:  # False -> Stop the program
                if program['process'] is not None:
                    self.log_and_publish(f'Stopping program: {topic_name}')
                    self.stop_program(topic_name)
                else:
                    self.log_and_publish(f'Program {topic_name} is not running.')

    def update_arguments_callback(self, msg):
        """Update arguments for a program dynamically."""
        try:
            # Message format: topic_name,arg_key,arg_value
            topic_name, arg_key, arg_value = msg.data.split(',')
            if topic_name in self.topics_programas:
                with self.process_lock:
                    program = self.topics_programas[topic_name]
                    if arg_key in program['arguments']:
                        self.log_and_publish(f'Updating argument {arg_key} for {topic_name} to {arg_value}')
                        program['arguments'][arg_key] = arg_value
                        if program['process'] is not None:
                            self.log_and_publish(f'Restarting program {topic_name} with updated arguments...')
                            self.stop_program(topic_name)
                            self.start_program(topic_name)
                    else:
                        self.log_and_publish(f'Argument {arg_key} not found for {topic_name}', 'warn')
            else:
                self.log_and_publish(f'Topic {topic_name} not found in programs.', 'warn')
        except ValueError:
            self.log_and_publish('Invalid format for update_arguments message. Expected "topic_name,arg_key,arg_value".', 'error')

    def start_program(self, topic_name):
        """Start a program based on its configuration.
        They can be:
        - Launch files: 'ros2 launch...'
        - Executables: 'ros2 run...
        - Terminal commands: 'exec...'
        """
        
        # ========= MAIN PROGRAM ===========
        program = self.topics_programas[topic_name]
        program.setdefault('process', None)
        
        if program['command'] == 'exec':
            cmd = program['executable_or_file'].split()
        elif program['command'] in ['launch', 'run']:
            cmd = ['ros2', program['command'], program['package'], program['executable_or_file']]
        else:
            self.log_and_publish(f'Unknown command type for {topic_name}: {program["command"]}', 'error')
            return
        
        # Handle sudo commands
        if "sudo" in cmd[0]:
            cmd = f'echo {PASSWORD} | sudo -S {" ".join(cmd[1:])}'
        else:
            cmd = " ".join(cmd)  # Ensure cmd is properly formatted as a string

        # ======== ARGUMENTS?? =============
        if program['arguments']:
            for arg_key, arg_value in program['arguments'].items():
                cmd.append(f'{arg_key}:={arg_value}')

        # ======== TRY TO START PROCESS ============
        self.log_and_publish(f'Executing command: {" ".join(cmd)}')

        process = subprocess.Popen(
            cmd,
            preexec_fn=os.setsid,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            shell=True
        )
        program['process'] = process
        self.log_and_publish(f'Process for {topic_name} trying to start with PID: {process.pid}')

        # ========== ERROR HANDLING / SET PROCESS TO STARTED ==========
        error_flag = {'detected': False}
        monitor_thread = Thread(target=self._capture_output, args=(process, topic_name, error_flag))
        monitor_thread.start()
        monitor_thread.join()  # Wait for the thread to finish

        # Check if an error was detected
        if error_flag['detected'] or process.returncode != 0:
            self.log_and_publish(f"Failed to start program: {topic_name}. Process not marked as running.", level='error')
            program['process'] = None
        else:
            self.log_and_publish(f"Program {topic_name} started successfully.")
    
    def _capture_output(self, process, topic_name, error_flag):
        """Capture and publish process output. Check for errors."""
        for line in process.stdout:
            self.log_and_publish(f'[{topic_name}][STDOUT]: {line.strip()}')

        for line in process.stderr:
            self.log_and_publish(f'[{topic_name}][STDERR]: {line.strip()}', level='error')
            # Detect error and set flag
            if "error" in line.lower() or "failed" in line.lower():
                error_flag['detected'] = True

        # Wait for the process to finish and check its exit code
        return_code = process.wait()
        if return_code != 0:
            self.log_and_publish(f'[{topic_name}] Process exited with return code {return_code}', level='error')
            error_flag['detected'] = True


    def stop_program(self, topic_name):
        """Stop the currently running process for the program."""
        program = self.topics_programas[topic_name]
        process = program['process']

        if process:
            self.log_and_publish(f'Stopping process for {topic_name}...')
            try:
                os.killpg(os.getpgid(process.pid), signal.SIGTERM)
                process.wait(timeout=5)
                self.log_and_publish(f'Process for {topic_name} terminated.')
            except subprocess.TimeoutExpired:
                # Y sí no es capaz por las buenas, se hace por las malas (SIGKILL, es decir, MATARRRRRRR, SANGREEEEEEEE)
                self.log_and_publish(f'Process for {topic_name} did not terminate in time. Killing...')
                os.killpg(os.getpgid(process.pid), signal.SIGKILL)
                self.log_and_publish(f'Process for {topic_name} killed.')
            except Exception as e:
                self.log_and_publish(f'Error stopping process for {topic_name}: {e}', 'error')
            program['process'] = None

    # ======== Remote Command Execution Methods ========

    def execute_command_callback(self, msg):
        """Callback to execute a terminal command received via the topic."""
        command = msg.data.strip()
        if not command:
            self.log_and_publish('Received empty command. Ignoring.', 'error')
            return

        self.log_and_publish(f'Executing remote command: {command}')
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
                self.log_and_publish(f'Command output: {result.stdout.strip()}')
            else:
                output_message.data = f'[ERROR]: {result.stderr.strip()}'
                self.log_and_publish(f'Command error: {result.stderr.strip()}', 'error')

            # Publish the result
            self.command_output_publisher.publish(output_message)

        except Exception as e:
            self.log_and_publish(f'Error executing command: {e}', 'error')
            error_message = String()
            error_message.data = f'[EXCEPTION]: {str(e)}'
            self.command_output_publisher.publish(error_message)

    def destroy_node(self):
        """Ensure all processes are stopped before shutting down."""
        self.log_and_publish('Shutting down supervisor node...')

        with self.process_lock:
            for topic_name, program in self.topics_programas.items():
                if program['process']:
                    self.log_and_publish(f'Terminating process for {topic_name}...')
                    try:
                        os.killpg(os.getpgid(program['process'].pid), signal.SIGTERM)
                        program['process'].wait(timeout=5)
                        self.log_and_publish(f'Process for {topic_name} terminated.')
                    except subprocess.TimeoutExpired:
                        self.log_and_publish(f'Process for {topic_name} did not terminate in time. Killing...', level='warn')
                        os.killpg(os.getpgid(program['process'].pid), signal.SIGKILL)
                        self.log_and_publish(f'Process for {topic_name} killed.')
                    except Exception as e:
                        self.log_and_publish(f'Error stopping process for {topic_name}: {e}', level='error')

                    # Ensure the process is cleared
                    program['process'] = None

        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = SupervisorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.log_and_publish('Keyboard interrupt received. Shutting down.')
    finally:
        node.destroy_node()
        rclpy.shutdown()
