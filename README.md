# ROS 2 TurtleBot3 Maze

基于 ROS 2 Humble、Gazebo Classic 和 TurtleBot3 Burger 的迷宫机器人综合实践项目。
项目完成了机器人仿真环境搭建、激光雷达数据处理、运动控制、左墙跟随和 Pure-Pursuit 航迹跟踪，并提供多张 Gazebo 迷宫地图用于测试。

课程代码：`(2026-2027-1)-S0610350-04`

## 功能

- 自定义矩形墙体迷宫 Gazebo 世界
- TurtleBot3 Burger 模型、里程计和二维激光雷达
- 开环运动控制、点到点控制和航点跟踪
- Pure-Pursuit 航迹跟踪节点
- 基础激光雷达避障和基础左墙跟随
- 优化左墙跟随：传感器 QoS、雷达扇区统计、墙距 P 控制、转向减速和左墙搜索
- RViz2 中的机器人模型、TF 和 LaserScan 可视化

## 环境

- Ubuntu 22.04
- ROS 2 Humble
- Gazebo Classic 11
- TurtleBot3 ROS 2 软件包

安装 TurtleBot3 相关软件包后，确认系统中存在以下资源：

```bash
ros2 pkg prefix turtlebot3_description
ros2 pkg prefix turtlebot3_gazebo
```

## 编译

本仓库根目录就是 ROS 2 工作空间目录，克隆后直接在仓库根目录编译：

```bash
cd ~/ros2maze_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

如果已有工作空间目录，需要将仓库中的 `src/maze_control` 复制或合并到该工作空间的 `src/` 目录，再在工作空间根目录执行以上命令。

## 一键启动

```bash
source /opt/ros/humble/setup.bash
source ~/ros2maze_ws/install/setup.bash
ros2 launch maze_control maze_tb3.launch.py
```

当前 launch 默认加载 `test_corridor.world`，机器人默认出生位置为 `(0.75, 0.75)`，并默认启动优化左墙跟随节点。

## 切换地图

地图文件位于 `src/maze_control/worlds/`。切换地图时，修改 `src/maze_control/launch/maze_tb3.launch.py` 中的 `world_file` 文件名，并根据地图入口调整 `spawn_robot` 的 `-x`、`-y` 和 `-z` 参数，然后重新编译：

```bash
cd ~/ros2maze_ws
colcon build --symlink-install
source install/setup.bash
ros2 launch maze_control maze_tb3.launch.py
```

## 可用节点

| 节点 | 作用 |
| --- | --- |
| `tb3_point2point` | 基于里程计的点到点控制 |
| `tb3_waypoints` | 航点序列跟踪 |
| `tb3_pure_pursuit` | Pure-Pursuit 航迹跟踪 |
| `scan_region` | 提取前方、左侧和右侧雷达距离 |
| `tb3_avoid` | 基础反应式避障 |
| `tb3_wall_follower` | 基础左墙跟随 |
| `tb3_wall_follower_optimized` | 优化左墙跟随 |

## 主要话题

- `/scan`：`sensor_msgs/msg/LaserScan` 激光雷达数据
- `/odom`：`nav_msgs/msg/Odometry` 里程计数据
- `/cmd_vel`：`geometry_msgs/msg/Twist` 速度控制指令
- `/tf`、`/tf_static`：机器人坐标变换

## 检查话题

```bash
ros2 topic list
ros2 topic echo /scan --once
ros2 topic echo /odom --once
ros2 topic info /scan --verbose
ros2 run rqt_graph rqt_graph
```

## 目录结构

```text
src/maze_control/
├── launch/          # ROS 2 Python launch 文件
├── maze_control/    # Python 控制节点
├── rviz/            # RViz2 配置
├── worlds/          # Gazebo 迷宫地图
├── package.xml
├── setup.py
└── setup.cfg
```

## 说明

本项目使用 TurtleBot3 软件包提供的机器人描述和 Gazebo 模型；仓库只包含本项目源代码、地图和配置，不包含 `build/`、`install/`、`log/` 等本机生成目录。

## 许可证

本项目采用 MIT License。TurtleBot3、ROS 2 和 Gazebo 相关软件包遵循其各自的许可证。
