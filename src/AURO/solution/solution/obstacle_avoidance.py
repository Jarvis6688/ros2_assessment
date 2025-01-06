import time
import math
import random
from geometry_msgs.msg import Twist

from . import constants
from .states import State

def _check_and_avoid_obstacles(self):
    """
    Check for obstacles and execute avoidance strategy if necessary.
    """
    front_blocked = self.scan_triggered[constants.SCAN_FRONT]
    left_blocked = self.scan_triggered[constants.SCAN_LEFT]
    right_blocked = self.scan_triggered[constants.SCAN_RIGHT]
    back_blocked = self.scan_triggered[constants.SCAN_BACK]

    if not any([front_blocked, left_blocked, right_blocked, back_blocked]):
        return False

    if front_blocked:
        self.get_logger().warn("Obstacle detected in front. Executing avoidance strategy.")
        last_turn = getattr(self, 'last_turn', None)

        if not left_blocked and not right_blocked:
            direction = constants.TURN_LEFT if last_turn != constants.TURN_LEFT else constants.TURN_RIGHT
            self.last_turn = direction
            self.get_logger().info(
                f"Both sides clear, randomly turning: {'Left' if direction == constants.TURN_LEFT else 'Right'} ({direction})")
            _smooth_turn(self, direction)
        elif not left_blocked:
            self.get_logger().info("Turning left.")
            _smooth_turn(self, constants.TURN_LEFT)
        elif not right_blocked:
            self.get_logger().info("Turning right.")
            _smooth_turn(self, constants.TURN_RIGHT)
        else:
            self.get_logger().warn("Front and sides blocked. Executing emergency maneuver.")
            _emergency_maneuver(self)

    elif left_blocked or right_blocked:
        _handle_side_obstacles(self, left_blocked, right_blocked)

    return True

def _handle_side_obstacles(self, left_blocked, right_blocked):
    """
    Adjust speed and direction to avoid side obstacles.
    """
    if left_blocked:
        self.get_logger().info("Obstacle too close on the left. Turning right.")
        _smooth_turn(self, constants.TURN_RIGHT)
    elif right_blocked:
        self.get_logger().info("Obstacle too close on the right. Turning left.")
        _smooth_turn(self, constants.TURN_LEFT)
    else:
        self.get_logger().warn("Obstacles on both sides. Reducing speed.")
        _reduce_speed(self)

def _reduce_speed(self, reduction_factor=0.5):
    """
    Reduce the robot's speed.
    """
    try:
        current_speed = getattr(self, 0.2, constants.LINEAR_VELOCITY)
        reduced_speed = current_speed * reduction_factor

        if reduced_speed < 0.05:
            reduced_speed = 0.05

        msg = Twist()
        msg.linear.x = reduced_speed
        self.cmd_vel_publisher.publish(msg)

        self.current_speed = reduced_speed
        self.get_logger().info(f"Speed reduced to: {self.current_speed:.2f} m/s")
    except Exception as e:
        self.get_logger().error(f"Error during speed reduction: {str(e)}")

def _smooth_turn(self, direction, base_speed=0.2):
    """
    Execute a smooth turn.
    """
    msg = Twist()
    turn_speed = min(0.5, max(0.2, 1.0 - getattr(self, 'front_distance', constants.SCAN_WARN_THRESHOLD) / constants.SCAN_WARN_THRESHOLD))

    msg.angular.z = turn_speed * direction
    msg.linear.x = base_speed * (1.0 - abs(msg.angular.z))
    self.cmd_vel_publisher.publish(msg)
    self.get_logger().info(
        f"Smooth turn executed: {'Left' if direction == constants.TURN_LEFT else 'Right'}, speed: {turn_speed:.2f}")

def _emergency_maneuver(self):
    """
    Execute emergency obstacle avoidance.
    """
    self.get_logger().warn("Emergency maneuver initiated.")

    backup_msg = Twist()
    backup_msg.linear.x = -0.15
    self.cmd_vel_publisher.publish(backup_msg)

    time.sleep(0.5)

    if not any(self.scan_triggered):
        self.get_logger().info("Environment clear. Resuming normal operation.")
        return

    _prepare_turn(self, 170, 180)

def _prepare_turn(self, min_angle, max_angle, force_direction=None):
    """
    Prepare for a turn by calculating angle and direction.
    """
    self.previous_yaw = self.yaw
    self.state = State.TURNING
    self.turn_angle = random.uniform(min_angle, max_angle)

    if force_direction is not None:
        self.turn_direction = force_direction
    else:
        left_dist = getattr(self, 'min_left_dist', 999.0) if not math.isinf(getattr(self, 'min_left_dist', 999.0)) else 999.0
        right_dist = getattr(self, 'min_right_dist', 999.0) if not math.isinf(getattr(self, 'min_right_dist', 999.0)) else 999.0

        if left_dist > right_dist + 0.1:
            chosen_dir = constants.TURN_LEFT
        elif right_dist > left_dist + 0.1:
            chosen_dir = constants.TURN_RIGHT
        else:
            chosen_dir = getattr(self, 'last_turn_direction', constants.TURN_LEFT)

        self.turn_direction = chosen_dir

    self.last_turn_direction = self.turn_direction

    turn_dir_str = "Left" if self.turn_direction == constants.TURN_LEFT else "Right"
    self.get_logger().info(f"Preparing to turn {self.turn_angle:.2f} degrees, direction: {turn_dir_str}")
