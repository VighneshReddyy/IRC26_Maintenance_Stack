from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():

    zed = ExecuteProcess(
        cmd=[
            'ros2','launch','zed_wrapper','zed.launch.py',
            '--ros-args','--log-level','warn'
        ],
        output='screen'
    )

    yolo = Node(
        package='yolo',
        executable='inference_engine_tiled',
        name='yolo_node',
        output='screen'
    )

    imu_conv = Node(
        package='planner',
        executable='imu_conversion_node',
        name='imu_conversion_node',
        output='log',              # fully silent on terminal
        arguments=['--ros-args','--log-level','fatal']
    )

    relay = Node(
        package='planner',
        executable='relay_node',
        name='relay_node',
        output='screen'
    )

    rtab = ExecuteProcess(
        cmd=['ros2','launch','planner','rtab.launch.py'],
        output='screen'
    )

    gps = Node(
        package='gps',
        executable='gps_rtk',
        name='gps_rtk_node',
        output='screen',
        arguments=['--ros-args','--log-level','warn']
    )

    return LaunchDescription([
        zed,
        yolo,
        imu_conv,
        relay,
        rtab,
        gps
    ])
