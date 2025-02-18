from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    # Declare Launch Arguments
    args_left_ns = DeclareLaunchArgument(
        'left_ns', default_value='/stereo/left',
        description='Namespace for the left camera'
    )
    args_left_port = DeclareLaunchArgument(
        'left_port', default_value='/dev/video2',
        description='Device port for the left camera'
    )
    args_right_ns = DeclareLaunchArgument(
        'right_ns', default_value='/stereo/right',
        description='Namespace for the right camera'
    )
    args_right_port = DeclareLaunchArgument(
        'right_port', default_value='/dev/video4',
        description='Device port for the right camera'
    )

    # Define Left Camera Node
    left_camera_node = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        name='left_camera',
        namespace=LaunchConfiguration('left_ns'),
        output='screen',
        parameters=[{
            'camera_device': LaunchConfiguration('left_port'),
        }],
    )

    # Define Right Camera Node
    right_camera_node = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        name='right_camera',
        namespace=LaunchConfiguration('right_ns'),
        output='screen',
        parameters=[{
            'camera_device': LaunchConfiguration('right_port'),
        }],
    )

    # Launch Description
    ld = LaunchDescription()

    # Add Launch Arguments
    ld.add_action(args_left_ns)
    ld.add_action(args_left_port)
    ld.add_action(args_right_ns)
    ld.add_action(args_right_port)

    # Add Camera Nodes
    ld.add_action(left_camera_node)
    ld.add_action(right_camera_node)

    return ld
