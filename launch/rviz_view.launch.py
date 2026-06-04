# Copyright (c) 2024 Intelligent Robotics Lab (URJC)
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

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
import launch_ros.descriptions
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    description_file = LaunchConfiguration('description_file')
    description_package = LaunchConfiguration('description_package')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_joint_state_publisher = LaunchConfiguration('use_joint_state_publisher')
    use_lowstate_joint_bridge = LaunchConfiguration('use_lowstate_joint_bridge')
    lowstate_topic = LaunchConfiguration('lowstate_topic')

    declared_arguments = [
        DeclareLaunchArgument(
            'description_package',
            default_value='go2_description',
            description='Package that contains the go2 URDF/XACRO model.'
        ),
        DeclareLaunchArgument(
            'description_file',
            default_value='go2_description.urdf',
            description='URDF/XACRO file name under <description_package>/urdf.'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Disable simulation time for local URDF visualization.'
        ),
        DeclareLaunchArgument(
            'use_joint_state_publisher',
            default_value='false',
            description='Set to true only if ros-joint-state-publisher is installed.'
        ),
        DeclareLaunchArgument(
            'use_lowstate_joint_bridge',
            default_value='true',
            description='Subscribe /lf/lowstate and publish /joint_states for go2 joints.'
        ),
        DeclareLaunchArgument(
            'lowstate_topic',
            default_value='/lf/lowstate',
            description='LowState topic name from which motor_state.q is read.'
        ),
    ]

    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]),
        ' ',
        PathJoinSubstitution([FindPackageShare(description_package), 'urdf', description_file]),
    ])

    robot_description_param = launch_ros.descriptions.ParameterValue(
        robot_description_content, value_type=str
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_description_param,
        }],
    )

    # Optional: only start joint_state_publisher if you have it installed.
    # Default false to avoid launch-time dependency on this package.
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        condition=IfCondition(use_joint_state_publisher),
    )

    # Relay /lf/lowstate.motor_state[i].q to /joint_states for URDF joints.
    lowstate_joint_bridge = Node(
        package='go2_user_tools',
        executable='lowstate_to_jointstate.py',
        name='lowstate_to_joint_state',
        output='screen',
        parameters=[{
            'lowstate_topic': lowstate_topic,
            'joint_state_topic': '/joint_states',
        }],
        condition=IfCondition(use_lowstate_joint_bridge),
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=[
            '-d',
            PathJoinSubstitution(
                [FindPackageShare('go2_user_tools'), 'config', 'go2_description_view.rviz']
            ),
        ],
    )

    return LaunchDescription(
        declared_arguments + [
            robot_state_publisher,
            joint_state_publisher,
            lowstate_joint_bridge,
            rviz,
        ]
    )
