# 导入系统模块
import sys
import time
import copy

# 导入 ROS2 的核心库
import rclpy  # ROS2 Python 客户端库
from rclpy.node import Node  # ROS2 节点基类
from rclpy.signals import SignalHandlerOptions  # 信号处理选项
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor  # 执行器和外部关闭异常
from rclpy.qos import QoSPresetProfiles  # QoS 预设配置
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup  # 回调组，用于控制并发执行

# 导入标准 ROS2 消息类型
from std_msgs.msg import Float32, Header
from geometry_msgs.msg import Twist, Pose, Point
from nav_msgs.msg import Odometry, OccupancyGrid, MapMetaData
from sensor_msgs.msg import LaserScan

# 导入自定义接口的消息和服务
from assessment_interfaces.msg import Item, ItemList  # 自定义消息类型
from auro_interfaces.msg import StringWithPose
from auro_interfaces.srv import ItemRequest  # 自定义服务类型

# 导入数学工具
from tf_transformations import euler_from_quaternion  # 四元数到欧拉角转换
import angles  # 角度计算工具
from enum import Enum  # 枚举类型，用于定义状态
import random  # 随机数生成器
import math  # 数学函数库
import numpy as np  # 数值计算库

# 12.11新添加导入必要的Nav2相关消息和动作类型
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from copy import deepcopy

# 定义机器人运动参数常量
LINEAR_VELOCITY = 0.3  # 机器人前进速度（米/秒）
ANGULAR_VELOCITY = 0.3  # 机器人转向速度（弧度/秒）

# 定义转向方向常量
TURN_LEFT = 1  # 向左转的角速度方向
TURN_RIGHT = -1  # 向右转的角速度方向

# 定义激光雷达参数
SCAN_THRESHOLD = 0.4  # 激光雷达检测距离阈值（米）
SCAN_WARN_THRESHOLD = 0.46  # 添加警告距离阈值，提前减速
SCAN_FRONT = 0  # 前方区域索引
SCAN_LEFT = 1  # 左侧区域索引
SCAN_BACK = 2  # 后方区域索引
SCAN_RIGHT = 3  # 右侧区域索引


# 定义机器人状态
class State(Enum):
    FORWARD = 0  # 向前行驶状态
    TURNING = 1  # 转向状态
    COLLECTING = 2  # 收集物品状态
    DROPPING = 3  # 放下物品状态
    DELIVERING_NAV2 = 4  # Use Nav2运送物品到区域状态


class RobotController(Node):
    def __init__(self):
        # 初始化节点
        super().__init__('robot_controller')

        # 初始化执行器引用
        self.executor = None

        # 初始化机器人状态变量
        self.state = State.FORWARD  # 初始状态为向前行驶
        self.pose = Pose()  # 当前位姿
        self.previous_pose = Pose()  # 上一次位姿
        self.yaw = 0.0  # 当前朝向角度
        self.previous_yaw = 0.0  # 上一次朝向角度
        self.turn_angle = 0.0  # 目标转向角度
        self.turn_direction = TURN_LEFT  # 转向方向
        self.goal_distance = random.uniform(1.0, 2.0)  # 目标行驶距离

        self.initial_pose = None

        self.min_left_dist = None
        self.min_right_dist = None
        self.last_turn_direction = None
        # 初始化传感器相关变量
        self.scan_triggered = [False] * 4  # 激光雷达触发标志
        self.items = ItemList()  # 检测到的物品列表
        self.item_held = False  # 是否持有物品
        self.held_item_color = None

        # 初始化扫描相关变量
        self.scan_start_time = None  # 扫描开始时间
        self.scan_duration = 2.0  # 扫描持续时间（秒）

        # 声明和获取ROS参数
        self.declare_parameter('robot_id', 'robot1')
        self.robot_id = self.get_parameter('robot_id').value

        # 创建回调组
        client_callback_group = MutuallyExclusiveCallbackGroup()
        timer_callback_group = MutuallyExclusiveCallbackGroup()
        # ---------------------------------位置订阅发布----------------------------------------

        self.navigator = BasicNavigator()
        # 设置导航参数

        self.initial_pose = PoseStamped()
        self.set_initial_pose()
        self.navigator.setInitialPose(self.initial_pose)
        self.navigator.waitUntilNav2Active()
        self.costmap_publisher = self.create_publisher(
            OccupancyGrid,
            'robot_position_grid',
            10
        )

        # 创建位置更新定时器
        self.position_timer = self.create_timer(
            0.5,  # 2Hz更新频率
            self.publish_position_update,
            callback_group=timer_callback_group
        )

        self.robot_radius = 0.3  # 机器人半径，根据实际情况调整

        # 声明导航相关的参数（如果需要）
        self.declare_parameter('max_vel_x', 0.3)
        self.declare_parameter('min_vel_x', -0.3)
        self.declare_parameter('max_vel_theta', 1.0)
        self.declare_parameter('min_vel_theta', -1.0)
        # ---------------------------位置订阅发布----------------------------------------------

        # 创建服务客户端
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

        # 创建订阅者
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

        # 创建发布者
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

        # 创建控制循环定时器
        self.timer_period = 0.1  # 10Hz
        self.timer = self.create_timer(
            self.timer_period,
            self.control_loop,
            callback_group=timer_callback_group
        )
        # 对于 robot2，添加一个状态标志来追踪是否已完成初始导航
        self.initial_navigation_done = False
        # 如果是 robot2，先进行初始导航
        if self.robot_id == 'robot2':
            self.state = State.DELIVERING_NAV2
            self.get_logger().info("Robot2 将先导航到指定位置")
        # 定义区域位置
        # self.zones = {
        #     'BOTTOM_RIGHT': {'color': 'RED', 'x': -3.42, 'y': -2.46},  # 右下角
        #     'TOP_RIGHT': {'color': 'RED', 'x': 2.37, 'y': -2.5},  # 右上角
        #     'TOP_LEFT': {'color': 'BLUE', 'x': 2.57, 'y': 2.46},  # 左上角
        #     'BOTTOM_LEFT': {'color': 'GREEN', 'x': -3.42, 'y': 2.46}  # 左下角
        # }
        # 定义区域位置和朝向
        self.zones = {
            'BOTTOM_RIGHT': {
                'color': 'RED',
                'x': -3.42,
                'y': -2.46,
                'target_yaw': math.pi / 20
            },
            'TOP_RIGHT': {
                'color': 'GREEN',
                'x': 2.37,
                'y': -2.5,
                'target_yaw': math.pi * 5 / 6
            },
            'TOP_LEFT': {
                'color': 'BLUE',
                'x': 2.37,
                'y': 2.46,
                'target_yaw': -3 * math.pi / 4
            },
            'BOTTOM_LEFT': {
                'color': 'BLACK',
                'x': -3.42,
                'y': 2.46,
                'target_yaw': -math.pi / 4
            }
        }

    def publish_position_update(self):
        """发布机器人位置作为占用栅格"""
        grid_msg = OccupancyGrid()
        grid_msg.header.frame_id = 'map'
        grid_msg.header.stamp = self.get_clock().now().to_msg()

        # 固定的栅格地图参数
        resolution = 0.05  # 5cm分辨率
        width = height = 40  # 2米范围 (40 * 0.05 = 2m)

        grid_msg.info.resolution = resolution
        grid_msg.info.width = width
        grid_msg.info.height = height

        # 设置原点（以机器人当前位置为中心）
        grid_msg.info.origin.position.x = self.pose.position.x - (width * resolution) / 2
        grid_msg.info.origin.position.y = self.pose.position.y - (height * resolution) / 2
        grid_msg.info.origin.orientation = self.pose.orientation

        # 创建占用栅格数据
        grid_msg.data = [-1] * (width * height)  # 初始化为未知状态

        # 将机器人位置标记为占用
        center_x = int(width / 2)
        center_y = int(height / 2)
        robot_radius = int(self.robot_radius / resolution)  # 机器人半径（格子数）

        # 将机器人占据的区域标记为已占用
        for i in range(-robot_radius, robot_radius + 1):
            for j in range(-robot_radius, robot_radius + 1):
                if i * i + j * j <= robot_radius * robot_radius:
                    idx = (center_y + j) * width + (center_x + i)
                    if 0 <= idx < len(grid_msg.data):
                        grid_msg.data[idx] = 100  # 设置为占用状态

        # 发布更新
        self.costmap_publisher.publish(grid_msg)
        self.get_logger().debug(f'Published position update for {self.robot_id}')

    def item_callback(self, msg):
        """处理物品检测消息的回调函数"""
        # 创建一个新的列表，用于保存过滤后的物品
        filtered_items = []

        for item in msg.data:
            # 如果是robot2，只关注绿球
            if self.robot_id == 'robot2':
                if item.colour.upper() == 'GREEN':
                    filtered_items.append(item)
                    self.get_logger().debug("Robot2 发现绿球")
            else:
                # 其他机器人正常处理
                filtered_items.append(item)

        # 将过滤后的物品列表赋值给 self.items
        self.items.data = filtered_items

        # 可以打印（或记录日志）看一下过滤后剩下多少物品
        if len(self.items.data) > 0:
            self.get_logger().debug(f"检测到 {len(self.items.data)} 个物品 (过滤后)")
        else:
            self.get_logger().debug("未检测到目标物品或已过滤")

    def odom_callback(self, msg):

        # 记录初始位置
        if self.initial_pose is None:
            self.initial_pose = msg.pose.pose

        """处理里程计消息的回调函数"""
        self.pose = msg.pose.pose
        # 将四元数转换为欧拉角
        (roll, pitch, yaw) = euler_from_quaternion([
            self.pose.orientation.x,
            self.pose.orientation.y,
            self.pose.orientation.z,
            self.pose.orientation.w
        ])
        self.yaw = yaw

    def scan_callback(self, msg):
        """处理激光雷达扫描消息的回调函数"""
        # 将扫描数据分为四个区域
        # front_ranges = msg.ranges[270:359] + msg.ranges[0:90]
        # left_ranges = msg.ranges[90:180]
        # back_ranges = msg.ranges[180:270]
        # right_ranges = msg.ranges[270:360]

        # 前方: 315°~360° + 0°~45°    共计 90°
        front_ranges = msg.ranges[315:360] + msg.ranges[0:45]

        # 左方: 45°~135°              共计 90°
        left_ranges = msg.ranges[45:135]

        # 后方: 135°~225°             共计 90°
        back_ranges = msg.ranges[135:225]

        # 右方: 225°~315°             共计 90°
        right_ranges = msg.ranges[225:315]

        # 过滤掉无效的测量值
        valid_front = [r for r in front_ranges if not math.isinf(r) and not math.isnan(r)]
        valid_left = [r for r in left_ranges if not math.isinf(r) and not math.isnan(r)]
        valid_right = [r for r in right_ranges if not math.isinf(r) and not math.isnan(r)]
        valid_back = [r for r in back_ranges if not math.isinf(r) and not math.isnan(r)]

        if valid_front:
            min_front = min(valid_front)

            # 如果当前检测到有颜色物品(即 items 列表不为空)
            if len(self.items.data) > 0:
                min_front = float('inf')

            self.scan_triggered[SCAN_FRONT] = min_front < SCAN_THRESHOLD
            if min_front < SCAN_WARN_THRESHOLD:
                self.front_distance = min_front
            else:
                self.front_distance = float('inf')

        if valid_left:
            self.scan_triggered[SCAN_LEFT] = min(valid_left) < SCAN_THRESHOLD
            self.min_left_dist = min(valid_left)
        else:
            self.min_left_dist = float('inf')
        if valid_right:
            self.scan_triggered[SCAN_RIGHT] = min(valid_right) < SCAN_THRESHOLD
            self.min_right_dist = min(valid_right)
        else:
            self.min_right_dist = float('inf')
        if valid_back:
            self.scan_triggered[SCAN_BACK] = min(valid_back) < SCAN_THRESHOLD

    def control_loop(self):
        """主控制循环 - 实现有限状态机"""
        # 发布当前状态到RViz
        marker_input = StringWithPose()
        marker_input.text = str(self.state)
        marker_input.pose = self.pose
        self.marker_publisher.publish(marker_input)

        # 对于 robot2 的特殊处理
        if self.robot_id == 'robot2' and not self.initial_navigation_done:
            if self.state == State.DELIVERING_NAV2:
                self._handle_initial_navigation()
                return
        # 状态机实现
        match self.state:
            case State.FORWARD:
                self._handle_forward_state()
            case State.TURNING:
                self._handle_turning_state()
            case State.COLLECTING:
                self._handle_collecting_state()
            case State.DROPPING:
                self._handle_attempt_offload()  # 新增放下物品状态处理
            case State.DELIVERING_NAV2:
                self._handle_delivering_state()

    def _handle_forward_state(self):
        """处理前进状态的逻辑"""

        if self.scan_triggered[SCAN_FRONT]:
            # 检测到前方障碍物，准备转向
            self._prepare_turn(90, 120)
            return

        if self.item_held:
            self.state = State.DELIVERING_NAV2
            self.get_logger().info(f"准备将{self.held_item_color}颜色的物品运送到对应区域")
            return

        # 检测可见的物品，计算距离并排序
        if len(self.items.data) > 0 and not self.item_held:

            nearest_item = self._compute_nearest_item(self.items.data)
            # 如果有物品比较近，切换到收集状态
            if nearest_item['distance'] < 2.0:
                if self._check_and_avoid_obstacles():
                    # 若有障碍物并已进行避障，则先不切状态，return结束
                    return
                # 若避障后仍安全，再进入COLLECTING
                self.state = State.COLLECTING
                return
        # 正常前进
        msg = Twist()
        msg.linear.x = LINEAR_VELOCITY
        self.cmd_vel_publisher.publish(msg)

        # 检查是否达到目标距离
        if self._check_and_avoid_obstacles():
            # 如果发现障碍物并已避让，结束当前循环
            return
        self.state = State.COLLECTING
        self.get_logger().info("到达目标距离，准备搜索物品")

    # ------------------------------robot2特殊逻辑开始------------------------------------------------
    def _handle_initial_navigation(self):
        """处理 robot2 的初始导航"""
        # 设置导航目标姿态
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.navigator.get_clock().now().to_msg()

        # 设置目标位置为 (0.0, 0.0)
        goal_pose.pose.position.x = 0.0
        goal_pose.pose.position.y = 0.0
        goal_pose.pose.position.z = 0.0

        # 设置朝向（这里设为0，表示朝向X轴正方向）
        yaw = 0.0
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        goal_pose.pose.orientation.x = 0.0
        goal_pose.pose.orientation.y = 0.0
        goal_pose.pose.orientation.z = sy
        goal_pose.pose.orientation.w = cy

        self.get_logger().info("Robot2 开始初始导航到 (0.0, 0.0)")
        self.navigator.goToPose(goal_pose)

        # 等待导航完成
        while not self.navigator.isTaskComplete():
            feedback = self.navigator.getFeedback()
            if feedback is not None and feedback.navigation_time.sec > 600:
                self.navigator.cancelNav()
                break

        # 处理导航结果
        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            self.get_logger().info("Robot2 已到达初始位置")
            self.initial_navigation_done = True
            self.state = State.FORWARD  # 切换到正常的前进状态
        elif result == TaskResult.CANCELED:
            self.get_logger().warn("Robot2 初始导航被取消")
            self.state = State.DELIVERING_NAV2  # 重试导航
        elif result == TaskResult.FAILED:
            self.get_logger().error("Robot2 初始导航失败")
            self.state = State.DELIVERING_NAV2  # 重试导航

    # ------------------------------robot2特殊逻辑结束------------------------------------------------

    def _handle_turning_state(self):
        """处理转向状态的逻辑"""
        msg = Twist()
        msg.angular.z = self.turn_direction * ANGULAR_VELOCITY
        self.cmd_vel_publisher.publish(msg)

        # 检查是否完成转向
        yaw_difference = angles.normalize_angle(self.yaw - self.previous_yaw)
        if math.fabs(yaw_difference) >= math.radians(self.turn_angle):
            self._complete_turn()

    def _handle_collecting_state(self):
        """处理收集物品状态的逻辑"""
        if len(self.items.data) == 0:
            _handle_scanning(self)
            return

        if self._check_and_avoid_obstacles():
            return

        # 计算所有可见物品的距离，并按距离排序
        nearest_item = self._compute_nearest_item(self.items.data)

        item = nearest_item['item']
        distance = nearest_item['distance']

        heading_error = item.x / 320.0

        self.current_target_item = nearest_item['item']
        if distance <= 0.35:
            if self._check_and_avoid_obstacles():
                # 如果避障逻辑触发，则结束当前循环
                return
            self._attempt_pickup()
        else:
            self._approach_item(distance, heading_error)

    def _compute_nearest_item(self, items):
        if not items:
            return None, None

        items_with_distance = []
        for item in items:
            distance = 32.4 * float(item.diameter) ** -0.75
            items_with_distance.append({
                'item': item,
                'distance': distance,
                'color': item.colour,
                'position': {'x': item.x, 'y': item.y}  # 添加位置信息
            })

        items_with_distance.sort(key=lambda x: x['distance'])
        nearest_item = items_with_distance[0]

        return nearest_item

    def _handle_scanning(self):

        if self._check_and_avoid_obstacles():
            return

        """处理扫描过程的逻辑"""
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
        """尝试拾取物品的逻辑"""
        # 停止机器人
        stop_msg = Twist()
        self.cmd_vel_publisher.publish(stop_msg)

        # 获取当前目标物品的颜色
        if self.current_target_item is not None:
            target_color = self.current_target_item.colour.upper()
        else:
            target_color = "UNKNOWN"

        # 创建并发送拾取请求
        rqt = ItemRequest.Request()
        rqt.robot_id = self.robot_id
        self.get_logger().info(f'当前拾取的机器ID：{self.robot_id}')

        try:
            future = self.pick_up_service.call_async(rqt)
            self.executor.spin_until_future_complete(future)
            response = future.result()

            if response.success:
                self.get_logger().info(f'{target_color}颜色的物品拾取成功')
                self.item_held = True
                self.held_item_color = target_color
                self.previous_pose = self.pose
                self.goal_distance = random.uniform(1.0, 2.0)
                self.state = State.FORWARD
            else:
                self.get_logger().warn(f'{target_color}颜色的物品拾取失败: {response.message}')
                self._handle_pickup_failure()

        except Exception as e:
            self.get_logger().error(f'拾取过程发生错误: {str(e)}')
            self.state = State.FORWARD

    def _handle_pickup_failure(self):
        """处理拾取失败的情况"""
        backup_msg = Twist()
        backup_msg.linear.x = -0.1
        self.cmd_vel_publisher.publish(backup_msg)
        self.state = State.FORWARD

    def _approach_item(self, distance, heading_error):
        """控制机器人接近物品"""
        msg = Twist()

        # 添加最小移动速度确保机器人不会停滞
        min_speed = 0.05

        # 根据前方障碍物距离调整速度
        if hasattr(self, 'front_distance') and self.front_distance < SCAN_WARN_THRESHOLD:
            speed_factor = max(0.3, self.front_distance / SCAN_WARN_THRESHOLD)
            msg.linear.x = max(min_speed, min(0.15, 0.2 * distance) * speed_factor)
            self.get_logger().info(f'检测到前方障碍物，减速至 {speed_factor:.2f}')
        else:
            msg.linear.x = max(min_speed, min(0.15, 0.2 * distance))

        # 增加转向灵敏度
        msg.angular.z = 0.8 * heading_error  # 从0.5增加到0.8

        self.cmd_vel_publisher.publish(msg)

    def _complete_turn(self):
        """完成转向动作"""
        self.previous_pose = self.pose
        self.goal_distance = random.uniform(1.0, 2.0)
        self.state = State.FORWARD
        self.get_logger().info(f"转向完成，开始前进 {self.goal_distance:.2f} 米")

    def _check_distance_reached(self):
        """检查是否达到目标距离"""
        dx = self.pose.position.x - self.previous_pose.position.x
        dy = self.pose.position.y - self.previous_pose.position.y
        distance_travelled = math.sqrt(dx * dx + dy * dy)
        return distance_travelled >= self.goal_distance

    def _handle_delivering_state(self):
        """处理运送物品到区域的逻辑"""
        # 1. 基础检查
        if not self.item_held:
            self.get_logger().debug("未持有物品，切换到前进状态")
            self.state = State.FORWARD
            return

        # 2. 获取目标区域
        target_zone = self.get_target_zone()
        self.get_logger().info(f"目的坐标信息: {target_zone}")
        if target_zone is None:
            self.get_logger().error(f"未找到{self.held_item_color}颜色对应的区域")
            self.state = State.FORWARD
            return

        # 3. 计算当前位置到目标的位置信息（用于日志和调试）
        dx = target_zone['x'] - self.pose.position.x
        dy = target_zone['y'] - self.pose.position.y
        distance = math.sqrt(dx * dx + dy * dy)
        target_angle = math.atan2(dy, dx)
        angle_diff = angles.normalize_angle(target_angle - self.yaw)

        # 4. 设置导航目标姿态
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.navigator.get_clock().now().to_msg()

        # 设置目标位置
        goal_pose.pose.position.x = target_zone['x']
        goal_pose.pose.position.y = target_zone['y']
        goal_pose.pose.position.z = 0.0

        # 使用区域预设的朝向
        yaw = target_zone['target_yaw']

        # 转换为四元数
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        goal_pose.pose.orientation.x = 0.0
        goal_pose.pose.orientation.y = 0.0
        goal_pose.pose.orientation.z = sy
        goal_pose.pose.orientation.w = cy

        # 5. 发送导航目标
        self.navigator.goToPose(goal_pose)

        # 等待导航完成，添加朝向检查
        while not self.navigator.isTaskComplete():
            feedback = self.navigator.getFeedback()
            if feedback is not None:
                # 检查当前朝向和目标朝向的差异
                current_yaw = self.yaw  # 假设self.yaw是当前朝向
                yaw_diff = abs(angles.normalize_angle(current_yaw - yaw))

                if feedback.navigation_time.sec > 600:
                    self.navigator.cancelNav()
                    break

        # 8. 处理导航结果
        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:

            self.state = State.DROPPING
        elif result == TaskResult.CANCELED:
            self.state = State.DELIVERING_NAV2
        elif result == TaskResult.FAILED:
            self.state = State.FORWARD

    def _handle_attempt_offload(self):
        """尝试在区域放下物品"""
        # 停止机器人
        stop_msg = Twist()
        self.cmd_vel_publisher.publish(stop_msg)

        # 创建并发送卸载请求
        rqt = ItemRequest.Request()
        rqt.robot_id = self.robot_id

        try:
            # 异步调用放置服务
            future = self.offload_service.call_async(rqt)
            self.executor.spin_until_future_complete(future)
            response = future.result()

            if response.success:
                self.get_logger().info(f'成功在{self.held_item_color}区域放下物品')
                self.item_held = False
                self.held_item_color = None

                # 执行安全的后退和转向逻辑
                # self._perform_safe_backup()
                self.get_logger().info('物品放置完成，准备切换状态寻找新物品')

                # 切换到前进状态
                self.state = State.FORWARD

            else:
                # 放置失败，记录失败信息并重新调整位置
                self.get_logger().warn(f'物品放置失败: {response.message}')
                self._handle_offload_failure()

        except Exception as e:
            # 异常处理，记录错误并尝试调整位置
            self.get_logger().error(f'放置过程发生错误: {str(e)}')
            self._handle_offload_failure()

    def _perform_safe_backup(self):
        """执行安全的后退和转向动作"""
        # 后退一定距离
        backup_msg = Twist()
        backup_msg.linear.x = -0.15
        self.cmd_vel_publisher.publish(backup_msg)
        time.sleep(1.0)  # 后退1秒

        # 转向远离目标区域
        self.previous_yaw = self.yaw
        self._prepare_turn(150, 180)

        # 记录新的起始位置
        self.previous_pose = self.pose
        self.goal_distance = random.uniform(0.5, 1.0)  # 设置较短的前进距离

    def _handle_offload_failure(self):
        """处理物品放置失败的情况"""
        self.get_logger().warn("放置失败，尝试调整位置并重新放置")

        # 后退一小段距离
        backup_msg = Twist()
        backup_msg.linear.x = -0.1
        self.cmd_vel_publisher.publish(backup_msg)
        time.sleep(0.5)  # 后退0.5秒

        # 随机调整角度重新尝试
        self.previous_yaw = self.yaw
        self._prepare_turn(90, 120)
        self.previous_pose = self.pose

        # 设置状态为放置，重新尝试
        self.state = State.DROPPING

    def destroy_node(self):
        """清理并销毁节点"""
        try:
            # 停止机器人
            stop_msg = Twist()
            self.cmd_vel_publisher.publish(stop_msg)
            self.get_logger().info("正在停止机器人并清理资源")
        finally:
            super().destroy_node()

    def _check_and_avoid_obstacles(self):
        """优化后的避障检查方法，使用更智能的避障策略"""
        # 检查四个方向的障碍物情况
        front_blocked = self.scan_triggered[SCAN_FRONT]
        left_blocked = self.scan_triggered[SCAN_LEFT]
        right_blocked = self.scan_triggered[SCAN_RIGHT]
        back_blocked = self.scan_triggered[SCAN_BACK]

        # 如果没有障碍物，直接返回
        if not any([front_blocked, left_blocked, right_blocked, back_blocked]):
            return False

        # 避障逻辑
        if front_blocked:
            self.get_logger().warn("前方有障碍物，执行避障策略")
            last_turn = getattr(self, 'last_turn', None)

            if not left_blocked and not right_blocked:
                direction = TURN_LEFT if last_turn != TURN_LEFT else TURN_RIGHT
                self.last_turn = direction
                # 左右均无障碍物时，随机选择转向方向
                self.get_logger().info(
                    f"左右均无障碍物，随机选择转向方向: {'左转' if direction == TURN_LEFT else '右转'} ({direction})")
                self._smooth_turn(direction)
            elif not left_blocked:
                # 左侧可行时，左转
                self.get_logger().info("左侧可行时，左转")
                self._smooth_turn(TURN_LEFT)
            elif not right_blocked:
                self.get_logger().info("右侧可行时，右转")
                # 右侧可行时，右转
                self._smooth_turn(TURN_RIGHT)
            else:
                # 左右均被阻挡，后退并大角度调整方向
                self.get_logger().warn("前方和两侧均被阻挡，执行后退")
                self._emergency_maneuver()

        elif left_blocked or right_blocked:
            # 动态调整速度与方向，防止侧面碰撞
            self._handle_side_obstacles(left_blocked, right_blocked)

        return True

    def _handle_side_obstacles(self, left_blocked, right_blocked):
        """处理侧面障碍物的动态避让"""
        if left_blocked:
            self.get_logger().info("左侧障碍物过近，向右调整")
            self._smooth_turn(TURN_RIGHT)
        elif right_blocked:
            self.get_logger().info("右侧障碍物过近，向左调整")
            self._smooth_turn(TURN_LEFT)
        else:
            self.get_logger().warn("两侧均有障碍物，小幅减速")
            self._reduce_speed()

    def _reduce_speed(self, reduction_factor=0.5):
        """
        减速方法：通过减小线速度来实现动态减速
        :param reduction_factor: 减速系数，默认减速至当前速度的 50%
        """
        try:
            # 假设当前速度已经存储
            current_speed = getattr(self, 0.2, LINEAR_VELOCITY)
            reduced_speed = current_speed * reduction_factor

            # 确保减速速度不会低于安全最小值
            if reduced_speed < 0.05:  # 最小安全速度
                reduced_speed = 0.05

            # 发布减速指令
            msg = Twist()
            msg.linear.x = reduced_speed
            self.cmd_vel_publisher.publish(msg)

            # 更新当前速度
            self.current_speed = reduced_speed
            self.get_logger().info(f"已减速，当前速度: {self.current_speed:.2f} m/s")
        except Exception as e:
            self.get_logger().error(f"减速时发生错误: {str(e)}")

    def _smooth_turn(self, direction, base_speed=0.2):
        """执行平滑转向"""
        msg = Twist()
        # 根据障碍物距离动态调整转向速度
        if hasattr(self, 'front_distance'):
            turn_speed = min(0.5, max(0.2, 1.0 - self.front_distance / SCAN_WARN_THRESHOLD))
        else:
            turn_speed = 0.3

        msg.angular.z = turn_speed * direction
        msg.linear.x = base_speed * (1.0 - abs(msg.angular.z))  # 转向时适当降低前进速度
        self.cmd_vel_publisher.publish(msg)
        self.get_logger().info(f"执行平滑转向，方向: {'左' if direction == TURN_LEFT else '右'}, 速度: {turn_speed:.2f}")

    def _emergency_maneuver(self):
        """处理紧急情况的避障动作"""
        self.get_logger().warn("执行紧急避障操作")

        # 首先尝试后退
        backup_msg = Twist()
        backup_msg.linear.x = -0.15
        self.cmd_vel_publisher.publish(backup_msg)

        # 等待短暂时间
        time.sleep(0.5)

        # 检查周围环境是否已经安全
        if not any(self.scan_triggered):
            self.get_logger().info("环境已安全，恢复正常运行")
            return
        # 如果仍然不安全，执行大角度转向
        self._prepare_turn(170, 180)

    def _prepare_turn(self, min_angle, max_angle, force_direction=None):
        self.previous_yaw = self.yaw
        self.state = State.TURNING
        # 随机角度
        self.turn_angle = random.uniform(min_angle, max_angle)
        # 如果指定了强制方向，则直接用
        if force_direction is not None:
            self.turn_direction = force_direction
        else:
            # 先根据激光距离判断左右空旷度
            left_dist = self.min_left_dist if not math.isinf(self.min_left_dist) else 999.0
            right_dist = self.min_right_dist if not math.isinf(self.min_right_dist) else 999.0

            if left_dist > right_dist + 0.1:
                # 明显左边更空
                chosen_dir = TURN_LEFT
            elif right_dist > left_dist + 0.1:
                # 明显右边更空
                chosen_dir = TURN_RIGHT
            else:
                # 左右差不多
                chosen_dir = self.last_turn_direction or TURN_LEFT

            self.turn_direction = chosen_dir

        # 保存本次转向方向，减少后面抖动
        self.last_turn_direction = self.turn_direction

        # 打印日志
        turn_dir_str = "左" if self.turn_direction == TURN_LEFT else "右"
        self.get_logger().info(
            f"准备转向 {self.turn_angle:.2f} 度，方向：{turn_dir_str}"
        )

    def set_initial_pose(self):
        """
        Set the initial pose of the robot based on its ID.
        """
        # Define initial poses for each robot
        robot_initial_positions = {
            "robot1": {"x": -3.5, "y": 2.0},
            "robot2": {"x": -3.5, "y": 0.0},
            "robot3": {"x": -3.5, "y": -2.0}
        }
        # Default frame ID
        frame_id = "map"
        # Check if robot ID exists in the positions dictionary
        if self.robot_id in robot_initial_positions:
            self.initial_pose.header.frame_id = frame_id
            self.initial_pose.header.stamp = self.navigator.get_clock().now().to_msg()
            self.initial_pose.pose.position.x = robot_initial_positions[self.robot_id]["x"]
            self.initial_pose.pose.position.y = robot_initial_positions[self.robot_id]["y"]
            self.initial_pose.pose.orientation.w = self.pose.orientation.w

        else:
            raise ValueError(f"Unknown robot ID: {self.robot_id}")

    def get_target_zone(self):

        """
        根据机器人持有物品的颜色匹配固定的区域。
        """
        # 遍历所有区域，查找颜色匹配的区域
        for zone_name, zone_info in self.zones.items():
            if zone_info.get('color') == self.held_item_color:
                # 找到匹配的区域，直接返回
                self.get_logger().info(f"找到匹配的区域: {zone_name} 对应颜色 {self.held_item_color}")
                return zone_info

        # 如果没有找到匹配的区域，返回 None
        self.get_logger().warn(f"没有找到匹配颜色 {self.held_item_color} 的区域")
        return None


def main(args=None):
    """主函数"""
    # 初始化ROS2
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)

    # 创建节点和执行器
    node = RobotController()
    executor = MultiThreadedExecutor()

    # 设置节点的执行器 引用
    node.executor = executor

    # 将节点添加到执行器
    executor.add_node(node)

    try:
        # 运行执行器
        executor.spin()
    except KeyboardInterrupt:
        # 处理Ctrl+C中断
        node.get_logger().info("收到键盘中断信号，正在关闭节点...")
    except ExternalShutdownException:
        # 处理外部关闭信号
        node.get_logger().error("收到外部关闭信号")
        sys.exit(1)
    finally:
        # 清理资源
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()