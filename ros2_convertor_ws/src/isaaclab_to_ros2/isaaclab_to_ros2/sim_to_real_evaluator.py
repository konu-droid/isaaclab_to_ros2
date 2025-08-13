#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
import message_filters
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
from rcl_interfaces.msg import ParameterDescriptor

class SimToRealEvaluator(Node):
    """
    A ROS 2 node to quantitatively measure the sim-to-real transfer fidelity.

    This node subscribes to the joint states of a simulated robot and a real robot,
    records their trajectories, and calculates a "Sim-to-Real Transfer Score"
    based on the Mean Squared Error (MSE) between the two time-series.
    """

    def __init__(self):
        """Initializes the node, parameters, subscribers, and services."""
        super().__init__('sim_to_real_evaluator')

        # --- Parameters ---
        # Declare parameters for the topic names to allow for easy configuration.
        self.declare_parameter(
            'sim_joint_topic', 
            '/sim_robot/joint_states',
            ParameterDescriptor(description='Topic for simulated robot joint states.')
        )
        self.declare_parameter(
            'real_joint_topic', 
            '/real_robot/joint_states',
            ParameterDescriptor(description='Topic for real robot joint states.')
        )
        sim_topic = self.get_parameter('sim_joint_topic').get_parameter_value().string_value
        real_topic = self.get_parameter('real_joint_topic').get_parameter_value().string_value

        # --- Data Storage ---
        self.sim_trajectory_data = []
        self.real_trajectory_data = []
        self.is_recording = False

        # --- Subscribers and Synchronizer ---
        # Subscribes to both topics and uses an ApproximateTimeSynchronizer to ensure
        # that messages from both sources are temporally aligned.
        self.get_logger().info(f"Listening to sim topic: '{sim_topic}'")
        self.get_logger().info(f"Listening to real topic: '{real_topic}'")
        
        sim_sub = message_filters.Subscriber(self, JointState, sim_topic)
        real_sub = message_filters.Subscriber(self, JointState, real_topic)

        # The 'slop' parameter allows for a small time difference between messages.
        self.time_synchronizer = message_filters.ApproximateTimeSynchronizer(
            [sim_sub, real_sub], queue_size=10, slop=0.1
        )
        self.time_synchronizer.registerCallback(self.synchronized_callback)

        # --- Services ---
        # Provides services to start recording and to stop and calculate the score.
        self.start_service = self.create_service(
            Trigger, 'start_recording', self.start_recording_callback
        )
        self.calculate_service = self.create_service(
            Trigger, 'calculate_transfer_score', self.calculate_score_callback
        )
        
        self.get_logger().info("Sim-to-Real Evaluator node is ready. Call '/start_recording' to begin.")

    def start_recording_callback(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        """Service callback to start recording the trajectories."""
        # Clear any previous data
        self.sim_trajectory_data.clear()
        self.real_trajectory_data.clear()
        
        # Set the recording flag to True
        self.is_recording = True
        
        self.get_logger().info("▶️ Started recording sim and real robot trajectories.")
        response.success = True
        response.message = "Recording started."
        return response

    def synchronized_callback(self, sim_msg: JointState, real_msg: JointState):
        """
        Callback for synchronized joint state messages.
        
        This method is called only when a message from both the sim and real topics
        with closely matching timestamps is received.
        """
        if self.is_recording:
            # For a fair comparison, ensure joint order is identical.
            # Here we assume the order is the same. If not, you would need
            # to reorder one of the arrays based on