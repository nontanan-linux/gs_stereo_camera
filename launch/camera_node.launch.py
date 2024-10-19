import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Declare launch arguments for device and namespace
    device = DeclareLaunchArgument(
        'device',
        default_value='0',
        description='Device port for the camera (e.g., /dev/video0)'
    )

    namespace = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Namespace for the camera node'
    )

    # Define the camera node
    camera_node = Node(
        package='gs_stereo_camera',
        executable='camera_node',
        namespace=LaunchConfiguration('namespace'),
        parameters=[{
            'device': LaunchConfiguration('device')
        }],
        output='screen'
    )

    # Launch Description
    ld = LaunchDescription()
    ld.add_action(device)
    ld.add_action(namespace)
    ld.add_action(camera_node)

    return ld
