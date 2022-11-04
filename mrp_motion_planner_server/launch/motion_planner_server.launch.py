from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction, OpaqueFunction)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def prepare_launch(context):
    robot_name_arg = DeclareLaunchArgument(
        'robot_name', default_value='')
    robot_name = LaunchConfiguration('robot_name')

    load_nodes = GroupAction(
        actions=[
            Node(
                package='multi_robot_component_testing',
                executable='minimal_service_server',
                output='screen',
                namespace=robot_name.perform(context),
                arguments=['--ros-args']
            ),
            Node(
                package='mrp_motion_planner_server',
                executable='motion_planner_server_exec',
                output='screen',
                namespace=robot_name.perform(context),
                arguments=['--ros-args'],
                parameters=[{'planner_name_list': ['spotturn',
                                                   'rvo'],
                             'planner_name_plugin_mapping': ['mrp_motion_planner::SpotturnController',
                                                             'mrp_motion_planner::RVO'],
                             'planner_plugin': 'rvo'}]
            )
        ]
    )

    return [
        robot_name_arg,
        load_nodes
    ]


def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=prepare_launch)
    ])
