import sys
import rclpy
from rclpy.executors import MultiThreadedExecutor, ExternalShutdownException
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions

from .robot_controller import RobotController

def main(args=None):
    # Initialize ROS2
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)

    # Create the RobotController node and executor
    node = RobotController()
    executor = MultiThreadedExecutor()

    # Assign the executor to the node
    node.executor = executor

    # Add the node to the executor
    executor.add_node(node)

    try:
        # Run the executor to handle callbacks
        executor.spin()
    except KeyboardInterrupt:
        # Handle Ctrl+C interruption
        node.get_logger().info("Keyboard interrupt received, shutting down node...")
    except ExternalShutdownException:
        # Handle external shutdown signals
        node.get_logger().error("External shutdown signal received")
        sys.exit(1)
    finally:
        # Clean up resources
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
