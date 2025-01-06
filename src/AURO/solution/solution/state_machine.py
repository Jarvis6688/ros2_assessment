import math
import random
import time

import angles

from assessment_interfaces.msg import Item, ItemList
from auro_interfaces.msg import StringWithPose
from auro_interfaces.srv import ItemRequest

from geometry_msgs.msg import Twist
from .states import State
from . import constants
from .obstacle_avoidance import (
    _check_and_avoid_obstacles,
    _prepare_turn
)
from .navigation_utils import _initialize_navigator, _navigate_to_goal


def _handle_forward_state(self):
    """Handle logic for the forward state."""
    if self.scan_triggered[constants.SCAN_FRONT]:
        # Obstacle detected in front, prepare to turn
        _prepare_turn(self, 90, 120)
        return

    if self.item_held:
        self.state = State.DELIVERING_NAV2
        self.get_logger().info(f"Preparing to deliver the {self.held_item_color} item to the designated area")
        return

    # Detect visible items, calculate distance, and sort
    if len(self.items.data) > 0 and not self.item_held:
        nearest_item = _compute_nearest_item(self, self.items.data)
        # If an item is relatively close, switch to collecting state
        if nearest_item['distance'] < 2.0:
            if _check_and_avoid_obstacles(self):
                # If obstacles are detected and avoidance is performed, exit the state
                return
            # If safe after avoidance, enter COLLECTING state
            self.state = State.COLLECTING
            return

    # Normal forward movement
    msg = Twist()
    msg.linear.x = constants.LINEAR_VELOCITY
    self.cmd_vel_publisher.publish(msg)

    # Check if the target distance is reached
    if _check_and_avoid_obstacles(self):
        # If an obstacle is detected and avoided, exit the current loop
        return
    self.state = State.COLLECTING


def _handle_turning_state(self):
    """Handle logic for the turning state."""
    msg = Twist()
    msg.angular.z = self.turn_direction * constants.ANGULAR_VELOCITY
    self.cmd_vel_publisher.publish(msg)

    yaw_difference = angles.normalize_angle(self.yaw - self.previous_yaw)
    if math.fabs(yaw_difference) >= math.radians(self.turn_angle):
        _complete_turn(self)


def _handle_collecting_state(self):
    """Handle logic for the collecting state."""
    if len(self.items.data) == 0:
        _handle_scanning(self)
        return
    if _check_and_avoid_obstacles(self):
        return

    # Calculate the distance to all visible items and sort by distance
    nearest_item = _compute_nearest_item(self, self.items.data)
    item = nearest_item['item']
    distance = nearest_item['distance']
    heading_error = item.x / 320.0  # Assuming image width of 640
    self.current_target_item = nearest_item['item']
    if distance <= 0.35:
        if _check_and_avoid_obstacles(self):
            # If avoidance logic is triggered, exit the current loop
            return
        _attempt_pickup(self)
    else:
        _approach_item(self, distance, heading_error)


def _compute_nearest_item(self, items):
    """Compute the nearest item from the list of items."""
    if not items:
        return None
    items_with_distance = []
    for item in items:
        distance = 32.4 * float(item.diameter) ** -0.75  # Example distance calculation
        items_with_distance.append({
            'item': item,
            'distance': distance,
            'color': item.colour,
            'position': {'x': item.x, 'y': item.y}  # Add position information
        })
    items_with_distance.sort(key=lambda x: x['distance'])
    nearest_item = items_with_distance[0]
    return nearest_item


def _handle_scanning(self):
    """Handle the scanning process logic."""
    if _check_and_avoid_obstacles(self):
        return
    current_time = self.get_clock().now()
    if self.scan_start_time is None:
        self.scan_start_time = current_time
    if (current_time - self.scan_start_time).nanoseconds < self.scan_duration * 1e9:
        msg = Twist()
        msg.angular.z = 0.3
        self.cmd_vel_publisher.publish(msg)
    else:
        self.scan_start_time = None
        self.state = State.FORWARD


def _attempt_pickup(self):
    """Attempt to pick up an item."""
    # Stop the robot
    stop_msg = Twist()
    self.cmd_vel_publisher.publish(stop_msg)
    # Get the color of the current target item
    target_color = self.current_target_item.colour.upper() if self.current_target_item else "UNKNOWN"
    # Create and send a pickup request
    request = ItemRequest.Request()
    request.robot_id = self.robot_id
    self.get_logger().info(f'Current pickup robot ID: {self.robot_id}')

    try:
        future = self.pick_up_service.call_async(request)
        self.executor.spin_until_future_complete(future)
        response = future.result()

        if response.success:
            self.get_logger().info(f'{self.robot_id}: Successfully picked up the {target_color} item')
            self.item_held = True
            self.held_item_color = target_color
            self.previous_pose = self.pose
            self.goal_distance = random.uniform(1.0, 2.0)
            self.state = State.FORWARD
        else:
            self.get_logger().warn(
                f'{self.robot_id}: Failed to pick up the {target_color} item: {response.message}')
            _handle_pickup_failure(self)

    except Exception as e:
        self.get_logger().error(f'Error during pickup process: {str(e)}')
        self.state = State.FORWARD


def _handle_pickup_failure(self):
    """Handle the case when pickup fails."""
    backup_msg = Twist()
    backup_msg.linear.x = -0.1
    self.cmd_vel_publisher.publish(backup_msg)
    self.state = State.FORWARD


def _approach_item(self, distance, heading_error):
    """Control the robot to approach the item."""
    msg = Twist()
    # Add a minimum movement speed to ensure the robot does not stall
    min_speed = 0.05
    # Adjust speed based on front obstacle distance
    if hasattr(self, 'front_distance') and self.front_distance < constants.SCAN_WARN_THRESHOLD:
        speed_factor = max(0.3, self.front_distance / constants.SCAN_WARN_THRESHOLD)
        msg.linear.x = max(min_speed, min(0.15, 0.2 * distance) * speed_factor)
        self.get_logger().info(f'Obstacle detected ahead, reducing speed to {speed_factor:.2f}')
    else:
        msg.linear.x = max(min_speed, min(0.15, 0.2 * distance))
    # Increase steering sensitivity
    msg.angular.z = 0.8 * heading_error  # Increased from 0.5 to 0.8
    self.cmd_vel_publisher.publish(msg)


def _complete_turn(self):
    """Complete the turning action."""
    self.previous_pose = self.pose
    self.goal_distance = random.uniform(1.0, 2.0)
    self.state = State.FORWARD
    self.get_logger().info(f"Turn completed, starting to move forward {self.goal_distance:.2f} meters")


def _handle_delivering_nav2_state(self):
    """Handle the logic for delivering items to a designated area."""
    # 1. Basic check
    if not self.item_held:
        self.get_logger().debug("No item held, switching to forward state")
        self.state = State.FORWARD
        return

    # 2. Get target zone
    target_zone = _get_target_zone(self)
    self.get_logger().info(f"Target zone coordinates: {target_zone}")
    if target_zone is None:
        self.get_logger().error(f"No designated area found for color {self.held_item_color}")
        self.state = State.FORWARD
        return

    # Assume zone_info has x, y, target_yaw
    x = target_zone['x']
    y = target_zone['y']
    yaw = target_zone['target_yaw']
    success = _navigate_to_goal(self, x, y, yaw)

    if success:
        self.state = State.DROPPING
    else:
        self.state = State.DELIVERING_NAV2


def _handle_attempt_offload(self):
    """Attempt to drop off the held item at the designated area."""
    # Stop the robot
    stop_msg = Twist()
    self.cmd_vel_publisher.publish(stop_msg)
    # Create and send an offload request
    request = ItemRequest.Request()
    request.robot_id = self.robot_id

    try:
        # Asynchronously call the offload service
        future = self.offload_service.call_async(request)
        self.executor.spin_until_future_complete(future)
        response = future.result()

        if response.success:
            self.get_logger().info(f'Successfully dropped off the item in the {self.held_item_color} area')
            self.item_held = False
            self.held_item_color = None

            self.get_logger().info('Item drop-off complete, preparing to switch state to find new items')

            # Switch to forward state
            self.state = State.FORWARD

        else:
            # Drop-off failed, log the failure and adjust position
            self.get_logger().warn(f'Item drop-off failed: {response.message}')
            _handle_offload_failure(self)

    except Exception as e:
        # Handle exceptions, log error, and attempt to adjust position
        self.get_logger().error(f'Error during drop-off process: {str(e)}')
        _handle_offload_failure(self)


def _handle_offload_failure(self):
    """Handle the case when drop-off fails."""
    self.get_logger().warn("Drop-off failed, attempting to adjust position and retry")
    # Move back a short distance
    backup_msg = Twist()
    backup_msg.linear.x = -0.1
    self.cmd_vel_publisher.publish(backup_msg)
    time.sleep(0.5)  # Move back for 0.5 seconds
    # Randomly adjust angle and retry
    self.previous_yaw = self.yaw
    _prepare_turn(self, 90, 120)
    self.previous_pose = self.pose
    # Set state to DROPPING to retry
    self.state = State.DROPPING


def _get_target_zone(self):
    """
    Match the robot's held item's color to a predefined zone.
    """
    # Iterate through all zones to find a color match
    for zone_name, zone_info in self.zones.items():
        if zone_info.get('color') == self.held_item_color:
            # Found a matching zone, return it
            self.get_logger().info(f"Found matching zone: {zone_name} for color {self.held_item_color}")
            return zone_info
    # If no matching zone is found, return None
    self.get_logger().warn(f"No zone found matching color {self.held_item_color}")
    return None
