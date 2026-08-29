from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    serial_port = LaunchConfiguration('serial_port')
    initial_ctrl = LaunchConfiguration('initial_ctrl')
    odom_frame_id = LaunchConfiguration('odom_frame_id')
    base_frame_id = LaunchConfiguration('base_frame_id')
    namespace = LaunchConfiguration('namespace')

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'serial_port',
                default_value='/dev/ttyUSB0',
                description='STM32 UART device',
            ),
            DeclareLaunchArgument(
                'initial_ctrl',
                default_value='0',
                description='Initial ctrl byte: bit0=enable, bit1=soft emergency stop',
            ),
            DeclareLaunchArgument(
                'odom_frame_id',
                default_value='odom',
                description='Frame ID used by the odometry message',
            ),
            DeclareLaunchArgument(
                'base_frame_id',
                default_value='base_link',
                description='Child frame ID used by the odometry message',
            ),
            DeclareLaunchArgument(
                'namespace',
                default_value='',
                description='Optional ROS namespace for the node and its topics',
            ),
            Node(
                package='robot_drive',
                executable='base_drive',
                name='base_drive',
                namespace=namespace,
                output='screen',
                emulate_tty=True,
                parameters=[
                    {
                        'serial_port': ParameterValue(serial_port, value_type=str),
                        'initial_ctrl': ParameterValue(initial_ctrl, value_type=int),
                        'odom_frame_id': ParameterValue(odom_frame_id, value_type=str),
                        'base_frame_id': ParameterValue(base_frame_id, value_type=str),
                    }
                ],
            ),
        ]
    )
