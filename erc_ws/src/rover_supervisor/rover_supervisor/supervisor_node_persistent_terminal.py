import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import subprocess
import threading


class SupervisorNode(Node):
    def __init__(self):
        super().__init__('supervisor_node')

        self.get_logger().info('Supervisor node started. Listening for terminal commands.')

        # Persistent shell process
        self.shell_process = None
        self.start_persistent_shell()

        # Publisher for terminal output
        self.command_output_publisher = self.create_publisher(String, '/persistent_terminal/output', 10)

        # Subscriber for receiving commands
        self.create_subscription(String, '/persistent_terminal/input', self.handle_command, 10)

    def start_persistent_shell(self):
        """Start a persistent shell process."""
        try:
            # Start a shell process in a non-blocking mode
            self.shell_process = subprocess.Popen(
                ['bash'],  # Replace 'bash' with your preferred shell (e.g., 'sh', 'zsh', etc.)
                stdin=subprocess.PIPE,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                bufsize=1
            )
            self.get_logger().info('Persistent shell started.')
            
            # Start threads to read stdout and stderr from the shell
            threading.Thread(target=self.read_stdout, daemon=True).start()
            threading.Thread(target=self.read_stderr, daemon=True).start()
        except Exception as e:
            self.get_logger().error(f'Failed to start persistent shell: {e}')

    def handle_command(self, msg):
        """Send a command to the persistent shell."""
        command = msg.data.strip()
        if not command:
            self.get_logger().warning('Received an empty command. Ignoring.')
            return

        if self.shell_process and self.shell_process.stdin:
            try:
                self.shell_process.stdin.write(command + '\n')
                self.shell_process.stdin.flush()
                self.get_logger().info(f'Sent command to shell: {command}')
            except Exception as e:
                self.get_logger().error(f'Failed to send command to shell: {e}')
        else:
            self.get_logger().error('Persistent shell is not running.')

    def read_stdout(self):
        """Continuously read stdout from the shell and publish it."""
        try:
            for line in iter(self.shell_process.stdout.readline, ''):
                output_message = String()
                output_message.data = f'[STDOUT]: {line.strip()}'
                self.command_output_publisher.publish(output_message)
        except Exception as e:
            self.get_logger().error(f'Error reading from shell stdout: {e}')

    def read_stderr(self):
        """Continuously read stderr from the shell and publish it."""
        try:
            for line in iter(self.shell_process.stderr.readline, ''):
                output_message = String()
                output_message.data = f'[STDERR]: {line.strip()}'
                self.command_output_publisher.publish(output_message)
        except Exception as e:
            self.get_logger().error(f'Error reading from shell stderr: {e}')

    def destroy_node(self):
        """Ensure the persistent shell process is terminated."""
        self.get_logger().info('Shutting down SupervisorNode...')
        if self.shell_process:
            try:
                self.shell_process.terminate()
                self.shell_process.wait(timeout=5)
                self.get_logger().info('Persistent shell terminated.')
            except Exception as e:
                self.get_logger().error(f'Failed to terminate persistent shell: {e}')
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
