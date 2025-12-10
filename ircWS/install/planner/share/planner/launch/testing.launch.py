from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    zed_silent = ExecuteProcess(
        cmd=[
            'bash', '-c',
            "ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed2i > /dev/null 2>&1"
        ]
    )

    yolo_silent = ExecuteProcess(
        cmd=[
            'bash', '-c',
            "ros2 run yolo inference > /dev/null 2>&1"
        ]
    )

    relay_node = Node(
        package='planner',
        executable='relay',
        name='relay_node',
        output='screen'
    )

    imu_conversion_node = Node(
        package='planner',
        executable='imu_conversion_node',
        name='imu_conversion',
        output='screen'
    )

    return LaunchDescription([
        zed_silent,
        yolo_silent,
        relay_node,
        imu_conversion_node
    ])
