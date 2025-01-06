import math
import time

from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from xpra.x11.bindings.window import constants

def _initialize_navigator(controller):
    """
    Initialize the Nav2 Navigator, set the initial pose, and wait for activation.
    :param controller: RobotController instance to access its attributes and methods
    """
    controller.navigator = BasicNavigator()
    controller.initial_pose = PoseStamped()
    _set_initial_pose(controller)
    controller.navigator.setInitialPose(controller.initial_pose)
    controller.navigator.waitUntilNav2Active()
    controller.get_logger().info("Navigator initialized and active")

def _navigate_to_goal(controller, x, y, yaw=0.0, frame_id='map') -> bool:
    """
    Navigate to the specified coordinates (x, y) with the given yaw.
    :param controller: RobotController instance
    :param x: Target X coordinate
    :param y: Target Y coordinate
    :param yaw: Target yaw (radians)
    :param frame_id: Coordinate frame, default is 'map'
    :return: True if successful, False otherwise
    """
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = frame_id
    goal_pose.header.stamp = controller.navigator.get_clock().now().to_msg()

    goal_pose.pose.position.x = x
    goal_pose.pose.position.y = y
    goal_pose.pose.position.z = 0.0

    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    goal_pose.pose.orientation.x = 0.0
    goal_pose.pose.orientation.y = 0.0
    goal_pose.pose.orientation.z = sy
    goal_pose.pose.orientation.w = cy

    controller.navigator.goToPose(goal_pose)

    while not controller.navigator.isTaskComplete():
        feedback = controller.navigator.getFeedback()
        if feedback and feedback.navigation_time.sec > 600:
            controller.navigator.cancelNav()

    result = controller.navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        controller.get_logger().info("Successfully navigated to goal.")
        return True
    elif result == TaskResult.CANCELED:
        controller.get_logger().warn("Navigation to goal was canceled.")
        return False
    else:
        controller.get_logger().error("Failed to navigate to goal.")
        return False

def _set_initial_pose(controller, frame_id='map'):
    """
    Set the initial pose of the robot based on its ID.
    :param controller: RobotController instance
    :param frame_id: Coordinate frame, default is 'map'
    """
    robot_initial_positions = {
        "robot1": {"x": -3.5, "y": 2.0},
        "robot2": {"x": -3.5, "y": 0.0},
        "robot3": {"x": -3.5, "y": -2.0}
    }

    if controller.robot_id in robot_initial_positions:
        controller.initial_pose.header.frame_id = frame_id
        controller.initial_pose.header.stamp = controller.navigator.get_clock().now().to_msg()
        controller.initial_pose.pose.position.x = robot_initial_positions[controller.robot_id]["x"]
        controller.initial_pose.pose.position.y = robot_initial_positions[controller.robot_id]["y"]
        controller.initial_pose.pose.orientation.w = controller.pose.orientation.w
    else:
        raise ValueError(f"Unknown robot ID: {controller.robot_id}")
