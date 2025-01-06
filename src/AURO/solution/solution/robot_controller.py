import copy
import math
import random
import sys
import time
from enum import Enum

import angles
import numpy as np
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from rclpy.signals import SignalHandlerOptions
from tf_transformations import euler_from_quaternion

from assessment_interfaces.msg import Item, ItemList
from auro_interfaces.msg import StringWithPose
from auro_interfaces.srv import ItemRequest
from geometry_msgs.msg import Point, Pose, PoseStamped, Twist
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from nav_msgs.msg import MapMetaData, Odometry, OccupancyGrid
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32, Header

# Local module imports
from . import constants
from .lidar_utils import process_lidar_data
from .navigation_utils import _initialize_navigator, _navigate_to_goal
from .states import State
from .state_machine import (
    _handle_forward_state,
    _handle_turning_state,
    _handle_collecting_state,
    _handle_attempt_offload,
    _handle_delivering_nav2_state
)


class RobotController(Node):
    def __init__(self):
        # Initialize the node
        super().__init__('robot_controller')

        # Initialize executor reference
        self.executor = None
        # Initialize robot state variables
        self.state = State.FORWARD  # Initial state is moving forward
        self.pose = Pose()  # Current pose
        self.previous_pose = Pose()  # Previous pose
        self.yaw = 0.0  # Current orientation angle
        self.previous_yaw = 0.0  # Previous orientation angle
        self.turn_angle = 0.0  # Target turn angle
        self.turn_direction = constants.TURN_LEFT  # Turn direction
        self.goal_distance = random.uniform(1.0, 2.0)  # Target travel distance
        self.initial_pose = None
        self.min_left_dist = None
        self.min_right_dist = None
        self.last_turn_direction = None
        # Initialize sensor-related variables
        self.scan_triggered = [False] * 4  # Laser scan trigger flags
        self.items = ItemList()  # List of detected items
        self.item_held = False  # Whether an item is being held
        self.held_item_color = None
        # Initialize scan-related variables
        self.scan_start_time = None  # Scan start time
        self.scan_duration = 2.0  # Scan duration (seconds)
        # Declare and get ROS parameters
        self.declare_parameter('robot_id', 'robot1')
        self.robot_id = self.get_parameter('robot_id').value
        # Create callback groups
        client_callback_group = MutuallyExclusiveCallbackGroup()
        timer_callback_group = MutuallyExclusiveCallbackGroup()

        # Initialize Nav2Navigator, set initial pose, and wait for activation
        _initialize_navigator(self)
        # --------------------------------- Position Subscription and Publishing ----------------------------------------
        self.costmap_publisher = self.create_publisher(
            OccupancyGrid,
            'robot_position_grid',
            10
        )
        # Create a timer for position updates
        self.position_timer = self.create_timer(
            0.5,  # 2Hz update frequency
            self.publish_position_update,
            callback_group=timer_callback_group
        )
        self.robot_radius = 0.3  # Robot radius, adjust as necessary
        # Declare navigation-related parameters (if needed)
        self.declare_parameter('max_vel_x', 0.3)
        self.declare_parameter('min_vel_x', -0.3)
        self.declare_parameter('max_vel_theta', 1.0)
        self.declare_parameter('min_vel_theta', -1.0)
        # --------------------------- Position Subscription and Publishing ----------------------------------------------

        # Create service clients
        self.pick_up_service = self.create_client(
            ItemRequest,
            '/pick_up_item',
            callback_group=client_callback_group
        )
        self.offload_service = self.create_client(
            ItemRequest,
            '/offload_item',
            callback_group=client_callback_group
        )
        # Create subscribers
        self.item_subscriber = self.create_subscription(
            ItemList,
            'items',
            self.item_callback,
            10,
            callback_group=timer_callback_group
        )
        self.odom_subscriber = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10,
            callback_group=timer_callback_group
        )
        self.scan_subscriber = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            QoSPresetProfiles.SENSOR_DATA.value,
            callback_group=timer_callback_group
        )
        # Create publishers
        self.cmd_vel_publisher = self.create_publisher(
            Twist,
            'cmd_vel',
            10
        )
        self.marker_publisher = self.create_publisher(
            StringWithPose,
            'marker_input',
            10,
            callback_group=timer_callback_group
        )
        # Create control loop timer
        self.timer_period = 0.1  # 10Hz
        self.timer = self.create_timer(
            self.timer_period,
            self.control_loop,
            callback_group=timer_callback_group
        )
        # Initialize robot work position
        if self.robot_id == 'robot2':
            self.state = State.INITIAL_NAVIGATION_ROBOT
        # Initialize defined zone positions and orientations
        self.zones = constants.ZONES

    def publish_position_update(self):
        """Publish the robot's position as an occupancy grid."""
        grid_msg = OccupancyGrid()
        grid_msg.header.frame_id = 'map'
        grid_msg.header.stamp = self.get_clock().now().to_msg()
        # Fixed grid map parameters
        resolution = 0.05  # 5cm resolution
        width = height = 40  # 2m range (40 * 0.05 = 2m)
        grid_msg.info.resolution = resolution
        grid_msg.info.width = width
        grid_msg.info.height = height
        # Set origin (centered at the robot's current position)
        grid_msg.info.origin.position.x = self.pose.position.x - (width * resolution) / 2
        grid_msg.info.origin.position.y = self.pose.position.y - (height * resolution) / 2
        grid_msg.info.origin.orientation = self.pose.orientation
        # Create occupancy grid data
        grid_msg.data = [-1] * (width * height)  # Initialize as unknown
        # Mark the robot's position as occupied
        center_x = int(width / 2)
        center_y = int(height / 2)
        robot_radius = int(self.robot_radius / resolution)  # Robot radius in cells
        # Mark the area occupied by the robot
        for i in range(-robot_radius, robot_radius + 1):
            for j in range(-robot_radius, robot_radius + 1):
                if i * i + j * j <= robot_radius * robot_radius:
                    idx = (center_y + j) * width + (center_x + i)
                    if 0 <= idx < len(grid_msg.data):
                        grid_msg.data[idx] = 100  # Set as occupied

        # Publish the update
        self.costmap_publisher.publish(grid_msg)
        self.get_logger().debug(f'Published position update for {self.robot_id}')

    def item_callback(self, msg):
        """Callback function to handle item detection messages."""
        # Create a new list to store filtered items
        filtered_items = []

        for item in msg.data:
            # If robot2, only focus on green balls
            if self.robot_id == 'robot2':
                if item.colour.upper() == 'GREEN':
                    filtered_items.append(item)
                    self.get_logger().debug("Robot2 detected a green ball")
            else:
                # Other robots handle items normally
                filtered_items.append(item)

        # Assign the filtered items to self.items
        self.items.data = filtered_items

        if len(self.items.data) > 0:
            self.get_logger().debug(f"Detected {len(self.items.data)} items (filtered)")
        else:
            self.get_logger().debug("No target items detected or all items filtered")

    def odom_callback(self, msg):
        """Callback function to handle odometry messages."""
        # Record initial position
        if self.initial_pose is None:
            self.initial_pose = msg.pose.pose

        self.pose = msg.pose.pose
        # Convert quaternion to Euler angles
        (roll, pitch, yaw) = euler_from_quaternion([
            self.pose.orientation.x,
            self.pose.orientation.y,
            self.pose.orientation.z,
            self.pose.orientation.w
        ])
        self.yaw = yaw

    def scan_callback(self, msg):
        """Callback function to handle laser scan messages."""
        scan_result = process_lidar_data(msg, self.items)
        self.front_distance = scan_result["front_dist"]
        self.min_left_dist = scan_result["left_dist"]
        self.min_right_dist = scan_result["right_dist"]

        self.scan_triggered[constants.SCAN_FRONT] = scan_result["front_triggered"]
        self.scan_triggered[constants.SCAN_LEFT] = scan_result["left_triggered"]
        self.scan_triggered[constants.SCAN_BACK] = scan_result["back_triggered"]
        self.scan_triggered[constants.SCAN_RIGHT] = scan_result["right_triggered"]

    def control_loop(self):
        """Main control loop implementing a finite state machine."""
        # Publish current state to RViz
        marker_input = StringWithPose()
        marker_input.text = str(self.state)
        marker_input.pose = self.pose
        self.marker_publisher.publish(marker_input)

        # State machine implementation
        match self.state:
            case State.FORWARD:
                _handle_forward_state(self)
            case State.TURNING:
                _handle_turning_state(self)
            case State.COLLECTING:
                _handle_collecting_state(self)
            case State.DROPPING:
                _handle_attempt_offload(self)
            case State.DELIVERING_NAV2:
                _handle_delivering_nav2_state(self)
            case State.INITIAL_NAVIGATION_ROBOT:
                _navigate_to_goal(self, 0.0, 0.0, 0.0)
                self.state = State.FORWARD

    def destroy_node(self):
        """Clean up and destroy the node."""
        try:
            # Stop the robot
            stop_msg = Twist()
            self.cmd_vel_publisher.publish(stop_msg)
            self.get_logger().info("Stopping robot and cleaning up resources")
        finally:
            super().destroy_node()
