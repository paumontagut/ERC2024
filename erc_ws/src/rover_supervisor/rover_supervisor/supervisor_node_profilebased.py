# Este programa no se basa en true/false, se basa en strings de los programas.
# La cosa es que usa otro fichero eexterno que evita hacer el bringup, ya que este código se encarga de ejecutar los programas que hagan falta.

# rover_supervisor/supervisor_node.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import subprocess
from rover_supervisor.config.task_profiles import TASK_LAUNCH_PROFILES

class SupervisorNode(Node):
    def __init__(self):
        super().__init__('supervisor_node')

        self.get_logger().info("Supervisor Node initialized")
        
        # Store active tasks and processes
        self.current_task = None
        self.active_processes = {}

        # Subscriber for task commands
        self.create_subscription(String, "/supervisor/task", self.task_callback, 10)

    def task_callback(self, msg):
        task = msg.data
        self.get_logger().info(f"Received task: {task}")

        if task not in TASK_LAUNCH_PROFILES:
            self.get_logger().error(f"Task {task} is not defined.")
            return

        self.switch_task(task)

    def switch_task(self, task):
        # Stop all current processes
        self.stop_all_processes()

        # Start new processes for the task
        self.current_task = task
        launch_profiles = TASK_LAUNCH_PROFILES[task]

        for topic, config in launch_profiles.items():
            self.start_process(topic, config)

    def start_process(self, topic, config):
        """Start a sensor using its predefined launch file."""
        self.get_logger().info(f"Starting {topic} with config: {config}")

        # Construct the ros2 launch command dynamically
        cmd = ["ros2", "launch"]
        if topic == "/gui/logitech_cameras":
            cmd += ["rover_bringup", "logitech_cameras.launch.py"]
            for key, value in config.items():
                cmd.append(f"{key}:={value}")
        elif topic == "/gui/unitree_lidar":
            cmd += ["rover_bringup", "unitree_lidar.launch.py"]
        elif topic == "/gui/realsense_camera":
            cmd += ["rover_bringup", "realsense_camera.launch.py"]
        # Add more cases for other sensors

        # Start the process
        process = subprocess.Popen(cmd)
        self.active_processes[topic] = process

    def stop_all_processes(self):
        """Stop all active processes."""
        self.get_logger().info("Stopping all active processes")
        for topic, process in self.active_processes.items():
            self.get_logger().info(f"Stopping {topic}")
            process.terminate()
            process.wait()
        self.active_processes.clear()


def main(args=None):
    rclpy.init(args=args)
    node = SupervisorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
