import subprocess
import signal
import os

class ROS2Controller:
    def __init__(self, workspace_path="~/unitree_ws"):
        self.workspace_path = workspace_path
        self.node_process = None

    def source_ros2_workspace(self):
        """
        Generate the command to source the ROS 2 workspace.

        Returns:
            str: A command to source the ROS 2 workspace.
        """
        return f"source {self.workspace_path}/install/setup.bash"

    def start_node(self, package, executable, args=None):
        """
        Start a ROS 2 node using ros2 run.

        Args:
            package (str): The ROS 2 package name.
            executable (str): The node executable name.
            args (str, optional): Additional arguments to run the node. Defaults to None.

        Raises:
            Exception: If the node is already running.
        """
        if self.node_process is not None:
            raise Exception("A ROS 2 node is already running. Kill the current node before starting a new one.")

        source_cmd = self.source_ros2_workspace()
        ros2_cmd = f"ros2 run {package} {executable} {args or ''}"
        full_cmd = f"{source_cmd} && {ros2_cmd}"

        print(f"Starting ROS 2 node with: {full_cmd}")

        # Start the node in a subprocess
        self.node_process = subprocess.Popen(["bash", "-c", full_cmd], preexec_fn=os.setsid)

    def kill_node(self):
        """
        Kill the currently running ROS 2 node.

        Raises:
            Exception: If no node is currently running.
        """
        if self.node_process is None:
            raise Exception("No ROS 2 node is currently running.")

        print("Killing the ROS 2 node...")
        os.killpg(os.getpgid(self.node_process.pid), signal.SIGTERM)
        self.node_process = None
        print("ROS 2 node has been terminated.")

    def publish_to_topic(self, topic, msg_type, message):
        """
        Publish a message to a ROS 2 topic.

        Args:
            topic (str): The target topic name.
            msg_type (str): The ROS 2 message type (e.g., std_msgs/msg/String).
            message (str): The message to publish.

        Example Usage:
            obj.publish_to_topic("/active_gesture", "std_msgs/msg/String", '{data: "open_palm"}')
        """
        source_cmd = self.source_ros2_workspace()
        ros2_cmd = f"ros2 topic pub {topic} {msg_type} \"{message}\" --once"
        full_cmd = f"{source_cmd} && {ros2_cmd}"

        print(f"Publishing to topic with: {full_cmd}")
        subprocess.run(["bash", "-c", full_cmd], check=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
