# Isaac Lab to ROS 2 Interface

This repository provides the necessary tools to bridge Isaac Lab with ROS 2, primarily for the so10x arm. It includes a ROS 2 workspace (`ros2_convertor_ws`) to facilitate this communication and an Isaac Lab task for training.

## 🤖 Repositories

  * **`lerobot_cube_move`**: Contains a custom Isaac Lab task designed to train the so10x arm to perform a reaching task.
  * **`so-arm100-ros2`**: Contains the ROS 2 packages and configuration for controlling the so100 series arms.
  * **`ros2_convertor_ws`**: The main ROS 2 workspace that connects Isaac Lab simulations to the ROS 2 ecosystem.

## ✨ `ros2_convertor_ws` Package

This workspace contains the `isaaclab_to_ros2` package.

### Key Nodes

  * **`isaac_to_ros2.py`**: A script that translates data from the Isaac Lab simulation environment and publishes it to ROS 2 topics.
  * **`sim_to_real_evaluator.py`**: A utility node that calculates a "Sim-to-Real Transfer Score" by comparing the joint trajectories of the simulated robot and a real robot.

### How to Use the Evaluator Node

1.  **Build the Workspace**:
    ```bash
    colcon build
    source install/setup.bash
    ```
2.  **Run the Node**:
    ```bash
    ros2 run isaaclab_to_ros2 evaluator_node
    ```
3.  **Start Recording Trajectories**:
    ```bash
    ros2 service call /start_recording std_srvs/srv/Trigger
    ```
4.  **Calculate the Score**: After the robot has completed its task, stop recording and calculate the score.
    ```bash
    ros2 service call /calculate_transfer_score std_srvs/srv/Trigger
    ```