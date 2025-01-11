AURO Assessment ROS2 Package

This repository contains the simulation configuration for AURO. The mission objective is to design and implement an automated robotic system that uses the TurtleBot3 Waffle Pi robot to collect items in a simulation environment and store them in the appropriate area.

------------------------------------------------------
Technical Requirements

- ROS 2 Version: Humble Hawksbill
- Simulation Tool: Gazebo Classic 11
- Programming Language: Python, using rclpy client library
- Robot Model: TurtleBot3 Waffle Pi (supports up to 3 robots)

Based on the assessment, assessment_interfaces, auro_interfaces, tf_relay packages, the simulation configuration is modified only for the solution package.

------------------------------------------------------
How to Run the Code

1. Configure ROS 2 Environment
   Verify your Gazebo installation:
   source /opt/ros/humble/setup.bash

2. Extracting ZIP Files
   unzip submits.zip -d /*/yourpath

3. Install Dependencies
   Ensure the following ROS 2 packages and Python libraries are installed:
   - ROS 2 Packages: geometry_msgs, sensor_msgs, nav_msgs, nav2_simple_commander
   - Python Libraries: angles, numpy, rclpy, tf_transformations

4. Build the Workspace
   Use colcon to build the workspace:
   colcon build  or colcon build --symlink-install
   source install/setup.bash

5. Launch the Simulation
   Start the simulation environment with the default configuration:
   ros2 launch solution solution_nav2_launch.py

------------------------------------------------------
Simulation Environment Description

Items and Zones
- Items:
  Three colors are available: red, green, and blue.

- Zones:
  There are four zones located at the corners of the environment, with colors cyan, purple, green, and pink.

- Pick and Place Items:
  - Use the /pick_up_item service to pick up an item.
  - Use the /offload_item service to place an item. To place an item, the robot must be within the zone.
  - Each zone initially accepts items of any color. However, once an item of a specific color is placed in a zone, the zone will only accept items of the same color afterward.

------------------------------------------------------
Simulation Nodes

item_manager
Manages the picking and placing of items.

Services:
- /pick_up_item: Picks up an item.
- /offload_item: Places an item.

Topics:
- /item_log: Logs the statistics of collected items.
- /item_holders: Displays the items currently held by each robot.

item_sensor
Detects items using an RGB camera.

Topics:
- items: Provides a list of visible items.
- camera/image_items: Displays an annotated camera image showing visible items.

robot_sensor
Detects other robots using an RGB camera.

Topics:
- robots: Provides a list of visible robots.
- camera/image_robots: Displays an annotated camera image showing visible robots.

zone_sensor
Detects zones using an RGB camera.

Topics:
- zone: Provides a list of visible zones.
- camera/image_zone: Displays an annotated camera image showing visible zones.

------------------------------------------------------
Launch File Parameters

Number of Robots
Specifies the number of robots in the simulation.

Type: Integer
Command-Line Example:
ros2 launch solution solution_nav2_launch.py num_robots:=3

Random Seed
Controls the distribution of items in the simulation environment to ensure reproducibility.

Type: Integer
Example: random_seed:=0

Sensor Noise
Enables or disables noise for sensors such as cameras, LiDAR, and IMU.

Type: String ('true' or 'false')
Default: 'false'
Example: sensor_noise:=true

Obstacles
Toggles the inclusion of obstacles in the simulation environment.

Type: String ('true' or 'false')
Default: 'true'
Example: obstacles:=false

Initial Positions
Defines the initial positions of robots by specifying a configuration file.

Type: String (file path)
Default File: config/initial_poses.yaml
Example: Update the positions by modifying the file contents.

------------------------------------------------------
Code Module Overview

constants.py
Contains parameters for robot motion, Lidar thresholds, and zone definitions. Key constants include:
- Linear and angular velocity settings.
- Lidar scan thresholds for obstacle detection.
- Zone positions and color mappings.

data_logger.py
Responsible for logging robot operation data to a CSV file.
- Subscribes to the /item_log topic to record item collection information.
- Saves statistical data, such as the number and value of red, green, and blue items.

lidar_utils.py
Processes Lidar data to detect obstacles in front, back, left, and right directions.
- Splits Lidar data by direction.
- Calculates minimum distances and sets trigger flags.

navigation_utils.py
Handles robot navigation.
- Initializes the navigator and sets the initial position.
- Navigates to target points and processes feedback.

state_machine.py
Implements the finite state machine (FSM) for robot behavior. Main states include:
- FORWARD: Forward movement state.
- TURNING: Turning state.
- COLLECTING: Item collection state.
- DELIVERING_NAV2: Item delivery state.
- DROPPING: Item dropping state.

robot_controller.py
The main node that coordinates robot behavior.
- Manages FSM logic.
- Processes Lidar and camera data.
- Interacts with services /pick_up_item and /offload_item.

obstacle_avoidance.py
Handles obstacle avoidance logic.
- Detects obstacles in all directions and adjusts motion accordingly.
- Includes emergency obstacle avoidance and speed reduction functionalities.

------------------------------------------------------
Notes

Log Output:
ros2 launch solution solution_nav2_launch.py | tee namespace_log.txt