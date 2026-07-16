# Copyright (C) 2023 Open Source Robotics Foundation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""
This is a per-robot launch script.

This file only spawns one robot (state publisher, robot entity, slam_toolbox,
rviz) into the already-running simulation.
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace, SetRemap


def generate_launch_description() -> LaunchDescription:
    # Get the launch directory
    bringup_dir = get_package_share_directory('slam_toolbox_multi_robot_demo')
    launch_dir = os.path.join(bringup_dir, 'launch')
    sim_dir = get_package_share_directory('nav2_minimal_tb3_sim')
    slam_toolbox_dir = get_package_share_directory('slam_toolbox')
    # Create the launch configuration variables
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    # Launch configuration variables specific to simulation
    rviz_config_file = LaunchConfiguration('rviz_config_file')
    use_robot_state_pub = LaunchConfiguration('use_robot_state_pub')
    use_rviz = LaunchConfiguration('use_rviz')
    slam_params_file = LaunchConfiguration('slam_params_file')
    pose = {
        'x': LaunchConfiguration('x_pose', default='-2.00'),
        'y': LaunchConfiguration('y_pose', default='-0.50'),
        'z': LaunchConfiguration('z_pose', default='0.01'),
        'R': LaunchConfiguration('roll', default='0.00'),
        'P': LaunchConfiguration('pitch', default='0.00'),
        'Y': LaunchConfiguration('yaw', default='0.00'),
    }
    robot_name = LaunchConfiguration('robot_name')
    robot_sdf = LaunchConfiguration('robot_sdf')
    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]
    # Declare the launch arguments
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace', default_value='', description='Top-level namespace'
    )
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true',
    )
    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        'rviz_config_file',
        default_value=os.path.join(bringup_dir, 'rviz', 'nav2_default_view.rviz'),
        description='Full path to the RVIZ config file to use',
    )
    declare_use_robot_state_pub_cmd = DeclareLaunchArgument(
        'use_robot_state_pub',
        default_value='True',
        description='Whether to start the robot state publisher',
    )
    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz', default_value='True', description='Whether to start RVIZ'
    )
    declare_robot_name_cmd = DeclareLaunchArgument(
        'robot_name', default_value='turtlebot3_waffle', description='name of the robot'
    )
    declare_robot_sdf_cmd = DeclareLaunchArgument(
        'robot_sdf',
        default_value=os.path.join(sim_dir, 'urdf', 'gz_waffle.sdf.xacro'),
        description='Full path to robot sdf file to spawn the robot in gazebo',
    )
    declare_slam_params_file_cmd = DeclareLaunchArgument(
        'slam_params_file',
        # scan_topic set to a relative name ("scan" instead of "/scan")
        # see config/mapper_params_online_async.yaml in this package.
        default_value=os.path.join(
            bringup_dir, 'config', 'mapper_params_online_async.yaml'),
        description='Full path to the ROS2 parameters file for slam_toolbox',
    )
    urdf = os.path.join(sim_dir, 'urdf', 'turtlebot3_waffle.urdf')
    with open(urdf, 'r') as infp:
        robot_description = infp.read()
    start_robot_state_publisher_cmd = Node(
        condition=IfCondition(use_robot_state_pub),
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace=namespace,
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time, 'robot_description': robot_description}
        ],
        remappings=remappings,
    )
    # Standard slam_toolbox instance, one per robot.
    start_slam = GroupAction(
        actions=[
            PushRosNamespace(namespace),
            # remaps for namespaces
            SetRemap(src='/tf', dst='tf'),
            SetRemap(src='/tf_static', dst='tf_static'),
            
            SetRemap(src='/scan', dst='scan'),
            SetRemap(src='/map', dst='map'),
            SetRemap(src='/map_metadata', dst='map_metadata'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(slam_toolbox_dir, 'launch', 'online_async_launch.py')
                ),
                launch_arguments={
                    'slam_params_file': slam_params_file,
                    'use_sim_time': use_sim_time,
                }.items(),
            ),
        ]
    )
    rviz_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'rviz_launch.py')),
        condition=IfCondition(use_rviz),
        launch_arguments={
            'namespace': namespace,
            'use_sim_time': use_sim_time,
            'rviz_config': rviz_config_file,
        }.items(),
    )
    # Spawn this robot into the already-running Gazebo world.
    gz_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'spawn_tb3_no_clock.launch.py')),
        launch_arguments={'namespace': namespace,
                          'use_sim_time': use_sim_time,
                          'robot_name': robot_name,
                          'robot_sdf': robot_sdf,
                          'x_pose': pose['x'],
                          'y_pose': pose['y'],
                          'z_pose': pose['z'],
                          'roll': pose['R'],
                          'pitch': pose['P'],
                          'yaw': pose['Y']}.items())
    
    ld = LaunchDescription()
    
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(declare_use_robot_state_pub_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_robot_name_cmd)
    ld.add_action(declare_robot_sdf_cmd)
    ld.add_action(declare_slam_params_file_cmd)
    ld.add_action(gz_robot)
    
    ld.add_action(start_robot_state_publisher_cmd)
    ld.add_action(start_slam)
    ld.add_action(rviz_cmd)
    return ld
