# Copyright (c) 2023 LG Electronics.
# Copyright (c) 2024 Open Navigation LLC
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


import os
from pathlib import Path
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (AppendEnvironmentVariable, DeclareLaunchArgument, ExecuteProcess,
                            GroupAction, IncludeLaunchDescription, LogInfo,
                            OpaqueFunction, RegisterEventHandler)
from launch.conditions import IfCondition
from launch.event_handlers import OnShutdown
from launch.launch_context import LaunchContext
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution, PythonExpression
from launch_ros.actions import Node
import yaml


def generate_robot_actions(name: str = '', pose: dict[str, float] = {}) -> GroupAction:
    """Generate the actions to launch a robot with the given name and pose."""
    bringup_dir = get_package_share_directory('slam_toolbox_multi_robot_demo')
    launch_dir = os.path.join(bringup_dir, 'launch')
    use_rviz = LaunchConfiguration('use_rviz')
    rviz_config_file = LaunchConfiguration('rviz_config')
    use_robot_state_pub = LaunchConfiguration('use_robot_state_pub')

    # Define commands for launching the navigation instances.
    # NOTE: tb3_simulation_launch.py only spawns this robot into the
    # already-running (shared) Gazebo world started below by this parent
    # launch file. It does NOT start its own Gazebo server/world.
    group = GroupAction(
            [
                LogInfo(
                    msg=['Launching namespace=', name, ' init_pose=', str(pose),]
                ),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(launch_dir, 'rviz_launch.py')
                    ),
                    condition=IfCondition(use_rviz),
                    launch_arguments={
                        'namespace': TextSubstitution(text=name),
                        'rviz_config': rviz_config_file,
                    }.items(),
                ),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(bringup_dir, 'launch', 'tb3_simulation_launch.py')
                    ),
                    launch_arguments={
                        'namespace': name,
                        'use_sim_time': 'True',
                        'use_rviz': 'False',
                        'use_robot_state_pub': use_robot_state_pub,
                        'x_pose': TextSubstitution(text=str(pose.get('x', 0.0))),
                        'y_pose': TextSubstitution(text=str(pose.get('y', 0.0))),
                        'z_pose': TextSubstitution(text=str(pose.get('z', 0.0))),
                        'roll': TextSubstitution(text=str(pose.get('roll', 0.0))),
                        'pitch': TextSubstitution(text=str(pose.get('pitch', 0.0))),
                        'yaw': TextSubstitution(text=str(pose.get('yaw', 0.0))),
                        'robot_name': TextSubstitution(text=name),
                    }.items(),
                ),
                # Start global_odom to robot odom TF publisher
                # Static anchor: global_odom -> map (NOT -> odom).
                #   global_odom -> map (static, known at spawn)
                #        -> odom (live, drift-corrected by slam_toolbox)
                #        -> base_footprint 
                Node(
                    package='tf2_ros',
                    executable='static_transform_publisher',
                    name='global_map_tf_broadcaster',
                    namespace=TextSubstitution(text=name),
                    remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
                    parameters=[{'use_sim_time': True}],
                    arguments=[
                        '--x', str(pose.get('x', 0.0)),
                        '--y', str(pose.get('y', 0.0)),
                        '--z', str(pose.get('z', 0.0)),
                        '--roll', str(pose.get('roll', 0.0)),
                        '--pitch', str(pose.get('pitch', 0.0)),
                        '--yaw', str(pose.get('yaw', 0.0)),
                        '--frame-id', 'global_odom',
                        '--child-frame-id', 'map'
                    ]
                )
            ]
        )
    return group


def launch_robots(context: LaunchContext, *args, **kwargs) -> list:
    """
    Parse the 'robots' launch argument and build one GroupAction per robot.

    (ForEach not available on ROS 2 Humble/Jazzy)
    """
    robots_str = LaunchConfiguration('robots').perform(context).strip()
    log_settings = LaunchConfiguration('log_settings').perform(context)

    actions = []

    if not robots_str:
        actions.append(LogInfo(msg=['No robots provided in the robots launch argument.']))
        return actions

    try:
        robots_list = [yaml.safe_load(robot.strip()) for robot in
                       robots_str.split(';') if robot.strip()]
    except yaml.YAMLError as e:
        actions.append(LogInfo(msg=[f'Error parsing the robots launch argument: {e}']))
        return actions

    if log_settings.lower() == 'true':
        actions.append(LogInfo(msg=[f'number_of_robots={len(robots_list)}']))

    for robot in robots_list:
        name = robot.get('name', '')
        pose = robot.get('pose', {})
        actions.append(generate_robot_actions(name=name, pose=pose))

    return actions


def generate_launch_description() -> LaunchDescription:
    """
    Bring up the multi-robots with given launch arguments.

    Launch arguments consist of robot name(which is namespace) and pose for initialization.
    ex) robots:='{name: 'robot1', pose: {x: 1.0, y: 1.0, yaw: 1.5707}};
                 {name: 'robot2', pose: {x: 1.0, y: 1.0, yaw: 1.5707}}'
    """
    # Get the launch directory
    bringup_dir = get_package_share_directory('multi_robot_slam_toolbox_simulation')
    sim_dir = get_package_share_directory('nav2_minimal_tb3_sim')

    # Simulation settings
    world = LaunchConfiguration('world')
    headless = LaunchConfiguration('headless')
    use_simulator = LaunchConfiguration('use_simulator')

    # On this example all robots are launched with the same settings
    rviz_config_file = LaunchConfiguration('rviz_config')
    use_robot_state_pub = LaunchConfiguration('use_robot_state_pub')
    log_settings = LaunchConfiguration('log_settings')

    # Declare the launch arguments
    declare_world_cmd = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(sim_dir, 'worlds', 'tb3_sandbox.sdf.xacro'),
        description='Full path to world file to load',
    )

    declare_robots_cmd = DeclareLaunchArgument(
        'robots',
        default_value=(
            "{name: 'robot1', pose: {x: 0.5, y: 0.5, yaw: 0}};"
            "{name: 'robot2', pose: {x: -0.5, y: -0.5, z: 0, roll: 0, pitch: 0, yaw: 1.5707}}"
        ),
        description='Robots and their initialization poses in YAML format',
    )

    declare_simulator_cmd = DeclareLaunchArgument(
        'headless', default_value='False', description='Whether to execute gzclient)'
    )

    declare_use_simulator_cmd = DeclareLaunchArgument(
        'use_simulator',
        default_value='True',
        description='Whether to start the simulator',
    )

    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        'rviz_config',
        default_value=os.path.join(
            bringup_dir, 'rviz', 'nav2_default_view.rviz'),
        description='Full path to the RVIZ config file to use.',
    )

    declare_use_robot_state_pub_cmd = DeclareLaunchArgument(
        'use_robot_state_pub',
        default_value='True',
        description='Whether to start the robot state publisher',
    )

    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz', default_value='True', description='Whether to start RVIZ'
    )

    declare_log_settings_cmd = DeclareLaunchArgument(
        'log_settings', default_value='true', description='Whether to log settings'
    )

    # Start Gazebo with a single shared world. All robots are spawned
    # into this same instance from generate_robot_actions() above.
    world_sdf = tempfile.mktemp(prefix='nav2_', suffix='.sdf')
    world_sdf_xacro = ExecuteProcess(
        cmd=['xacro', '-o', world_sdf, ['headless:=', headless], world])
    start_gazebo_cmd = ExecuteProcess(
        cmd=['gz', 'sim', '-r', '-s', world_sdf],
        output='screen',
        condition=IfCondition(use_simulator),
    )

    start_gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'),
                         'launch',
                         'gz_sim.launch.py')
        ),
        condition=IfCondition(PythonExpression(
            [use_simulator, ' and not ', headless])),
        launch_arguments={'gz_args': ['-v4 -g ']}.items(),
    )

    remove_temp_sdf_file = RegisterEventHandler(event_handler=OnShutdown(
        on_shutdown=[
            OpaqueFunction(function=lambda _: os.remove(world_sdf))
        ]))

    # Bridge /clock, globally
    start_clock_bridge_cmd = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='clock_bridge',
        output='screen',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
    )

    set_env_vars_resources = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH', os.path.join(sim_dir, 'models'))
    set_env_vars_resources2 = AppendEnvironmentVariable(
            'GZ_SIM_RESOURCE_PATH',
            str(Path(os.path.join(sim_dir)).parent.resolve()))

    # Create the launch description and populate
    ld = LaunchDescription()
    ld.add_action(set_env_vars_resources)
    ld.add_action(set_env_vars_resources2)

    # Declare the launch options
    ld.add_action(declare_world_cmd)
    ld.add_action(declare_robots_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(declare_use_robot_state_pub_cmd)
    ld.add_action(declare_simulator_cmd)
    ld.add_action(declare_use_simulator_cmd)
    ld.add_action(declare_log_settings_cmd)

    # Add the actions to start gazebo (once, shared) then the robots
    ld.add_action(world_sdf_xacro)
    ld.add_action(start_gazebo_cmd)
    ld.add_action(start_gazebo_client)
    ld.add_action(remove_temp_sdf_file)
    ld.add_action(start_clock_bridge_cmd)

    ld.add_action(
        LogInfo(
            condition=IfCondition(log_settings),
            msg=['rviz config file: ', rviz_config_file],
        )
    )
    ld.add_action(
        LogInfo(
            condition=IfCondition(log_settings),
            msg=['using robot state pub: ', use_robot_state_pub],
        )
    )

    ld.add_action(OpaqueFunction(function=launch_robots))

    return ld
