
# 只启动相机与串口节点

import os
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # 定义参数文件路径
    params_file = DeclareLaunchArgument(
        'params_file',
        default_value=PathJoinSubstitution([FindPackageShare('auto_aim_bringup'), 'config', 'debug.yaml']),
        description='Full path to the parameter file for send_trans_processor_node'
    )

    hik_params_file = os.path.join(
        get_package_share_directory('hik_camera'), 'config', 'camera_params.yaml')

    camera_info_url = 'package://hik_camera/config/camera_info.yaml'

    return LaunchDescription([
        params_file,
        DeclareLaunchArgument(name='params_file',
                              default_value=hik_params_file),
        DeclareLaunchArgument(name='camera_info_url',
                              default_value=camera_info_url),
        DeclareLaunchArgument(name='use_sensor_data_qos',
                              default_value='false'),

        Node(
            package='hik_camera',
            executable='hik_camera_node',
            output='screen',
            emulate_tty=True,
            parameters=[LaunchConfiguration('params_file'), {
                'camera_info_url': LaunchConfiguration('camera_info_url'),
                'use_sensor_data_qos': LaunchConfiguration('use_sensor_data_qos'),
            }],
        ),

        Node(
            package='auto_aim',
            executable='serial_read_data_node',
            name='serial_read_data_node',
            parameters=[LaunchConfiguration('params_file')],
            output='screen',
        ),

    ])