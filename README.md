# AURO Assessment ROS2 Package

本仓库包含 AURO Assessment 的仿真配置。任务目标是设计和实现一个自动机器人系统，使用 TurtleBot3 Waffle Pi 机器人在仿真环境中收集物品并将其存放到适当的区域。

---

## 技术要求

- **ROS 2 版本**: Humble Hawksbill。
- **仿真工具**: Gazebo Classic 11。
- **编程语言**: Python，使用 `rclpy` 客户端库。
- **机器人模型**: TurtleBot3 Waffle Pi（最多支持 3 个机器人）。

仿真配置包括以下节点：
- `item_manager`：用于管理场地中的物品。
- 每个机器人包含的节点：`item_sensor`、`robot_sensor` 和 `zone_sensor`。

请注意，这些节点的实现不可更改。

---

## 如何运行代码

1. **配置 ROS 2 环境**
   确保已安装 ROS 2 和 Gazebo，并激活相关环境：
   ```bash
   source /opt/ros/humble/setup.bash
   ```

2. **克隆仓库**
   ```bash
   git clone <repository-url>
   cd <repository-folder>
   ```

3. **安装依赖**
   确保以下 ROS 2 包和 Python 库已安装：
   - ROS 2 包：`geometry_msgs`，`sensor_msgs`，`nav_msgs`，`nav2_simple_commander`
   - Python 库：`angles`，`numpy`，`rclpy`，`tf_transformations`

4. **构建工作区**
   使用 colcon 构建工作区：
   ```bash
   colcon build
   source install/setup.bash
   ```

5. **启动仿真**
   使用默认配置启动仿真环境：
   ```bash
   ros2 launch assessment assessment_launch.py
   ```

---

## 仿真环境说明

### 物品与区域
- **物品**：三种颜色（红、绿、蓝）。
- **区域**：四个角落的区域，颜色分别为青、紫、绿、粉。
- **拾取和放置物品**：
  - 使用 `/pick_up_item` 服务拾取物品。
  - 使用 `/offload_item` 服务放置物品。放置物品时，机器人必须位于区域内。
  - 每个区域初始接受任何颜色的物品，但一旦放置了某种颜色的物品，该区域后续只能接受相同颜色的物品。

### 仿真节点
- **`item_manager`**：管理物品的放置和拾取。
  - 服务：
    - `/pick_up_item`：拾取物品。
    - `/offload_item`：放置物品。
  - 话题：
    - `/item_log`：记录已收集的物品统计信息。
    - `/item_holders`：显示每个机器人当前持有的物品。

- **`item_sensor`**：通过 RGB 摄像头检测物品。
  - 话题：
    - `items`：可见物品列表。
    - `camera/image_items`：标注可见物品的摄像头图像。

- **`robot_sensor`**：通过 RGB 摄像头检测其他机器人。
  - 话题：
    - `robots`：可见机器人列表。
    - `camera/image_robots`：标注可见机器人的摄像头图像。

- **`zone_sensor`**：通过 RGB 摄像头检测区域。
  - 话题：
    - `zone`：可见区域列表。
    - `camera/image_zone`：标注可见区域的摄像头图像。

### 启动文件参数
使用 `assessment_launch.py` 启动仿真，可以通过以下参数调整仿真设置：
- **机器人数量**：
  ```bash
  ros2 launch assessment assessment_launch.py num_robots:=3
  ```
- **随机种子**：控制物品分布。
- **传感器噪声**：启用/禁用相机、激光雷达、IMU 的噪声。
- **障碍物**：启用/禁用场景中的障碍物。
- **初始位置**：通过配置文件 `config/initial_poses.yaml` 修改初始位置。

---

## 代码模块概述

### `constants.py`
包含机器人运动参数、Lidar 阈值和区域定义。主要常量包括：
- 线速度和角速度的设置。
- Lidar 扫描阈值，用于检测障碍物。
- 区域位置和颜色映射。

### `data_logger.py`
负责记录机器人操作数据到 CSV 文件中。
- 订阅 `/item_log` 话题以记录物品收集信息。
- 保存统计数据（如红、绿、蓝物品数量和价值）。

### `lidar_utils.py`
处理 Lidar 数据以检测前后左右的障碍物。
- 按方向分割 Lidar 数据。
- 计算最小距离并设置触发标志。

### `navigation_utils.py`
负责机器人导航。
- 初始化导航器并设置初始位置。
- 导航到目标点并处理反馈。

### `state_machine.py`
实现机器人有限状态机（FSM）。主要状态包括：
- `FORWARD`: 前进状态。
- `TURNING`: 转向状态。
- `COLLECTING`: 收集物品状态。
- `DELIVERING_NAV2`: 物品送达状态。
- `DROPPING`: 放置物品状态。

### `robot_controller.py`
主节点，协调机器人行为。
- 管理 FSM 逻辑。
- 处理 Lidar 和摄像头数据。
- 与服务 `/pick_up_item` 和 `/offload_item` 交互。

### `obstacle_avoidance.py`
处理避障逻辑。
- 检测前后左右障碍物并调整运动。
- 包括紧急避障和减速功能。

---

## 测试场景

### 场景 1：拾取物品
- 描述：机器人检测并拾取物品。
- 节点：`item_sensor`，`item_manager`
- 启动命令：
  ```bash
  ros2 launch assessment assessment_launch.py num_robots:=1
  ```

### 场景 2：避障导航
- 描述：机器人检测障碍物并调整路径。
- 节点：`zone_sensor`，`robot_sensor`
- 启动命令：
  ```bash
  ros2 launch assessment assessment_launch.py obstacles:=true
  ```

### 场景 3：物品分类送达
- 描述：机器人拾取物品并送达对应的区域。
- 节点：`item_manager`，`zone_sensor`
- 启动命令：
  ```bash
  ros2 launch assessment assessment_launch.py num_robots:=2
  ```

---

## 备注

- **运行日志**：
  ```bash
  ros2 launch assessment assessment_launch.py | tee simulation_log.txt
  ```
- **可视化**：
  通过 Gazebo 查看仿真环境，或启用 `camera/image_*` 话题进行调试。

如有疑问或需要帮助，请参考仓库中的文档或提出问题。
