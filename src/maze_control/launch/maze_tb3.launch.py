import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    maze_package = get_package_share_directory('maze_control')
    gazebo_package = get_package_share_directory('gazebo_ros')

    world_file = os.path.join(
        maze_package,
        'worlds',
        'test_corridor.world'
    )

    description_urdf = os.path.join(
        get_package_share_directory('turtlebot3_description'),
        'urdf',
        'turtlebot3_burger.urdf'
    )

    robot_sdf = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'),
        'models',
        'turtlebot3_burger',
        'model.sdf'
    )


    rviz_config = os.path.join(
        maze_package,
        'rviz',
        'maze_config.rviz'
    )

    with open(description_urdf, 'r', encoding='utf-8') as file:
        robot_description = file.read()

    # 当前系统提供的是普通 URDF，不是经过 xacro 展开的文件。
    # 删除未展开的 namespace 占位符，避免生成错误的 TF frame 名称。
    robot_description = robot_description.replace('${namespace}', '')

    run_controller = LaunchConfiguration('run_controller')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                gazebo_package,
                'launch',
                'gazebo.launch.py'
            )
        ),
        launch_arguments={
            'world': world_file
        }.items()
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {
                'robot_description': robot_description,
                'use_sim_time': True
            }
        ]
    )

    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_entity',
        output='screen',
        arguments=[
            '-file', robot_sdf,
            '-entity', 'turtlebot3_burger',
            # 基础地图小车初始坐标
            # '-x', '-3.4',
            # '-y', '-3.0',
            # '-z', '0.15'

            # 小车初始坐标，更换地图时需要更改
            '-x', '0.75',
            '-y', '0.75',
            '-z', '0.02'
        ]
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[
            {
                'use_sim_time': True
            }
        ]
    )

    wall_following = Node(
        package='maze_control',
        executable='tb3_wall_follower_optimized',
        name='wall_following',
        output='screen',
        condition=IfCondition(run_controller),
        parameters=[
            {
                'use_sim_time': True
            }
        ]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'run_controller',
            default_value='true',
            description='是否启动迷宫控制算法节点'
        ),
        gazebo,
        robot_state_publisher,
        spawn_robot,
        rviz,
        wall_following
    ])