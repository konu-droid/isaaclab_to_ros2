#!/usr/bin/env python3
import argparse
import os
import stat
from pathlib import Path

# --- Template for the generated ROS 2 Node (policy_node.py) ---
# This template contains the core logic for the ROS 2 node.
# It's populated with values from the command-line arguments.
POLICY_NODE_TEMPLATE = """
import rclpy
from rclpy.node import Node
import numpy as np

# Import necessary ROS 2 message types.
# You may need to add more imports depending on your specific robot.
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Header

# Placeholder for a library like PyTorch or TensorFlow.
# In a real scenario, you would import your deep learning framework.
# import torch

class PolicyModel:
    \"\"\"
    A placeholder class for your trained reinforcement learning policy.
    In a real implementation, this class would load the model weights
    from the specified path and handle inference.
    \"\"\"
    def __init__(self, policy_path):
        self.policy_path = policy_path
        print(f"Initializing model... (loading from {self.policy_path})")
        # Example: self.model = torch.load(self.policy_path)
        # Example: self.model.eval()
        print("Model loaded successfully (simulation).")

    def predict(self, observation_tensor):
        \"\"\"
        Simulates running inference on the observation tensor.
        Returns a raw action from the policy.
        \"\"\"
        print(f"Running inference on observation shape: {observation_tensor.shape}")
        # In a real scenario:
        # with torch.no_grad():
        #     raw_action = self.model(observation_tensor)
        # return raw_action.numpy()

        # For this placeholder, we return a dummy action of the correct size.
        # Assuming the policy outputs 1 action per joint.
        num_actions = observation_tensor.shape[0]
        dummy_action = np.random.randn(num_actions) * 0.5
        print(f"Simulated raw action: {dummy_action}")
        return dummy_action


class PolicyNode(Node):
    \"\"\"
    This ROS 2 node subscribes to robot state, processes it for the policy,
    gets an action, post-processes it, and publishes it as a motor command.
    \"\"\"
    def __init__(self, policy_path, state_topic, command_topic):
        super().__init__('isaac_policy_node')
        self.get_logger().info("Isaac Sim Policy-to-ROS 2 Node has started.")

        # 1. Model Loading
        self.get_logger().info(f"Loading policy from: {policy_path}")
        self.policy_model = PolicyModel(policy_path)
        self.joint_names = [] # To be populated from the first state message

        # 2. ROS 2 Subscriptions
        self.get_logger().info(f"Subscribing to robot state on: '{state_topic}'")
        self.subscription = self.create_subscription(
            JointState,
            state_topic,
            self.state_callback,
            10)

        # 3. ROS 2 Publishing
        self.get_logger().info(f"Publishing motor commands to: '{command_topic}'")
        self.publisher = self.create_publisher(
            JointTrajectory,
            command_topic,
            10)

    def state_callback(self, msg: JointState):
        \"\"\"
        The main callback that triggers on receiving a new robot state message.
        \"\"\"
        self.get_logger().debug(f"Received JointState with {len(msg.position)} joints.")
        
        if not self.joint_names:
            self.joint_names = msg.name
            self.get_logger().info(f"Registered joint names: {self.joint_names}")

        # 4. Data Preprocessing
        observation_tensor = self.preprocess_data(msg)

        # 5. Inference
        raw_action = self.policy_model.predict(observation_tensor)

        # 6. Action Postprocessing
        command_message = self.postprocess_action(raw_action)

        # 7. Publish Command
        self.publisher.publish(command_message)
        self.get_logger().debug("Published new joint trajectory command.")

    def preprocess_data(self, ros_message: JointState) -> np.ndarray:
        \"\"\"
        Transforms incoming ROS 2 messages into the precise tensor format
        required by the neural network.

        Args:
            ros_message: The message received from the state topic.

        Returns:
            A NumPy array formatted for the policy model.
        \"\"\"
        # --- USER IMPLEMENTATION REQUIRED ---
        # This is a critical step. The observation space from Isaac Lab must
        # be reconstructed here from ROS messages.
        # This might involve combining joint positions, velocities, TF data, etc.
        # For this example, we'll just use the joint positions.
        
        # Example:
        positions = np.array(ros_message.position, dtype=np.float32)
        # velocities = np.array(ros_message.velocity, dtype=np.float32)
        # observation = np.concatenate([positions, velocities])
        
        return positions

    def postprocess_action(self, raw_action: np.ndarray) -> JointTrajectory:
        \"\"\"
        Converts the raw action outputs from the policy into the appropriate
        ROS 2 message type for the robot's controller.

        Args:
            raw_action: The output from the policy model.

        Returns:
            A ROS 2 message ready to be published.
        \"\"\"
        # --- USER IMPLEMENTATION REQUIRED ---
        # The raw action (e.g., target joint positions, velocities, or torques)
        # must be converted into the specific message format your robot expects.
        # Here, we format it as a JointTrajectory message.

        command_msg = JointTrajectory()
        command_msg.header = Header(stamp=self.get_clock().now().to_msg())
        command_msg.joint_names = self.joint_names

        # Create a single trajectory point
        point = JointTrajectoryPoint()
        point.positions = list(raw_action)
        point.time_from_start.sec = 1 # Reach the target in 1 second

        command_msg.points.append(point)
        return command_msg


def main(args=None):
    rclpy.init(args=args)
    
    # These arguments would typically be passed from a launch file
    # or a configuration management system. For this generated script,
    # we hardcode them based on the converter's input.
    policy_path = "{policy_path}"
    state_topic = "{state_topic}"
    command_topic = "{command_topic}"

    policy_node = PolicyNode(policy_path, state_topic, command_topic)

    try:
        rclpy.spin(policy_node)
    except KeyboardInterrupt:
        pass
    finally:
        policy_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
"""

# --- Template for package.xml ---
PACKAGE_XML_TEMPLATE = """<?xml version="1.0"?>
<package format="3">
  <name>{package_name}</name>
  <version>0.0.1</version>
  <description>Auto-generated ROS 2 package for an Isaac Sim policy.</description>
  <maintainer email="user@example.com">user</maintainer>
  <license>Apache 2.0</license>

  <buildtool_depend>ament_python</buildtool_depend>

  <depend>rclpy</depend>
  <depend>std_msgs</depend>
  <depend>sensor_msgs</depend>
  <depend>trajectory_msgs</depend>

  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
"""

# --- Template for setup.py ---
SETUP_PY_TEMPLATE = """from setuptools import setup

package_name = '{package_name}'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@example.com',
    description='Auto-generated ROS 2 package for an Isaac Sim policy.',
    license='Apache 2.0',
    tests_require=['pytest'],
    entry_points={{
        'console_scripts': [
            'policy_node = {package_name}.policy_node:main',
        ],
    }},
)
"""

# --- Template for setup.cfg ---
SETUP_CFG_TEMPLATE = """[develop]
script_dir=$base/lib/{package_name}
[install]
install_scripts=$base/lib/{package_name}
"""

def create_ros2_package(args):
    """Main function to generate the ROS 2 package structure and files."""
    package_name = args.package_name
    output_dir = Path(args.output_dir)
    package_path = output_dir / package_name
    
    if package_path.exists():
        print(f"Error: Directory '{package_path}' already exists. Aborting.")
        return

    print(f"Creating ROS 2 package '{package_name}' at '{output_dir}'...")

    # Create directory structure
    src_path = package_path / package_name
    resource_path = package_path / "resource"
    src_path.mkdir(parents=True, exist_ok=True)
    resource_path.mkdir(exist_ok=True)

    # --- Create files from templates ---

    # 1. package.xml
    with open(package_path / "package.xml", "w") as f:
        f.write(PACKAGE_XML_TEMPLATE.format(package_name=package_name))

    # 2. setup.py
    with open(package_path / "setup.py", "w") as f:
        f.write(SETUP_PY_TEMPLATE.format(package_name=package_name))
        
    # 3. setup.cfg
    with open(package_path / "setup.cfg", "w") as f:
        f.write(SETUP_CFG_TEMPLATE.format(package_name=package_name))

    # 4. The main policy node
    node_script_path = src_path / "policy_node.py"
    with open(node_script_path, "w") as f:
        # Use the real path for the policy file in the generated script
        abs_policy_path = os.path.abspath(args.policy_path)
        f.write(POLICY_NODE_TEMPLATE.format(
            policy_path=abs_policy_path,
            state_topic=args.state_topic,
            command_topic=args.command_topic,
            package_name=package_name
        ))
    # Make the script executable
    node_script_path.chmod(node_script_path.stat().st_mode | stat.S_IEXEC)

    # 5. Other required files
    (src_path / "__init__.py").touch()
    (resource_path / package_name).touch()
    
    print("\n--- Generation Complete! ---")
    print(f"ROS 2 package created at: {package_path}")
    print("\nNext Steps:")
    print(f"1. Navigate to your ROS 2 workspace: `cd your_ros2_ws`")
    print(f"2. Copy the package: `cp -r {package_path} src/`")
    print(f"3. Build the workspace: `colcon build --packages-select {package_name}`")
    print(f"4. Source the workspace: `source install/setup.bash`")
    print(f"5. Run your new node: `ros2 run {package_name} policy_node`")
    print("\nNOTE: You must implement the data pre-processing and post-processing")
    print(f"logic in `{node_script_path}` to match your RL task.")


def main():
    parser = argparse.ArgumentParser(
        description="Isaac-to-ROS 2 Policy Conversion Script",
        formatter_class=argparse.RawTextHelpFormatter
    )
    parser.add_argument(
        "--policy-path",
        required=True,
        help="Path to the trained policy weights file (e.g., model.pth)."
    )
    parser.add_argument(
        "--task-def",
        required=True,
        help="Name or path of the Isaac Lab task definition (for user reference)."
    )
    parser.add_argument(
        "--state-topic",
        default="/joint_states",
        help="ROS 2 topic for subscribing to robot state."
    )
    parser.add_argument(
        "--command-topic",
        default="/joint_trajectory_controller/joint_trajectory",
        help="ROS 2 topic for publishing motor commands."
    )
    parser.add_argument(
        "--package-name",
        required=True,
        help="Name for the new ROS 2 package to be generated (e.g., 'my_robot_policy')."
    )
    parser.add_argument(
        "--output-dir",
        default=".",
        help="The directory where the new ROS 2 package will be created."
    )
    
    args = parser.parse_args()
    create_ros2_package(args)


if __name__ == "__main__":
    main()
