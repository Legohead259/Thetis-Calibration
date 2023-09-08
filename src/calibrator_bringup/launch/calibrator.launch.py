"""
ROS2 launch file for the Thetis calibrator machine

CHANGELOG:
 - Version 1.0.0: Initial release
"""

__author__      = "Braidan Duffy"
__copyright__   = "Copyright 2023"
__credits__     = "Braidan Duffy"
__license__     = "MIT"
__version__     = "1.0.0"
__maintainer__  = "Braidan Duffy"
__email__       = "bduffy2018@my.fit.edu"
__status__      = "Prototype"

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch_ros.actions import Node
from uuid import uuid4
from calibrator_common.common.parameters import ParameterNames


def generate_launch_description():     
    rosbag_uuid = str(uuid4())
    rosbag_path = f"bags/{rosbag_uuid}/"
        
    return LaunchDescription([      
        # Recorder initialize
        ExecuteProcess(
            cmd=['ros2', 'bag', 'record', '/inertial', '-o', rosbag_path],
            output="both"
        ),
        
        # Pause recorder immediately
        ExecuteProcess(
            cmd=["ros2", "service", "call", "/rosbag2_recorder/pause", "rosbag2_interfaces/srv/Pause"]
        ),
        
        # Spin up master node
        Node(
            package="calibrator_master",
            executable="master",
            parameters=[{
                ParameterNames.ROSBAG_UUID.value: rosbag_uuid,
                ParameterNames.ROSBAG_PATH.value: rosbag_path
            }]
        ),
        
        # Spin up calibrator plate node
        Node(
            package="calibrator_plate",
            executable="plate"
        ),
        
        # Spin up xio device node
        Node(
            package="xio_device",
            executable="XioDeviceNode",
            # arguments=['--ros-args', '--log-level', 'debug'],
        ),
        
        # Set motor speed to zero
        ExecuteProcess(
            cmd=['ros2', 'service', 'call', '/set_motor_speed', 'calibrator_interfaces/SetFloat64', '{data: 0.0}']
        ),
    ])