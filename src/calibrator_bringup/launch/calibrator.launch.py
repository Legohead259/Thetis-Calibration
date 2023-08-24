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

import launch
from launch_ros.actions import Node
from uuid import uuid4


def generate_launch_description():
    ld = launch.LaunchDescription()
    
    recorder_initialize = launch.actions.ExecuteProcess(
        cmd=['ros2', 'bag', 'record', '/inertial', '-o', f'bags/{uuid4()}'],
        output="both"
    )
    
    recorder_pause = launch.actions.ExecuteProcess(
        cmd=["ros2", "service", "call", "/rosbag2_recorder/pause", "rosbag2_interfaces/srv/Pause"]
    )
    
    motor_controller_node = Node(
        package="calibrator_plate",
        executable="drv8825"
    )

    master_node = Node(
        package="calibrator_master",
        executable="master"
    )
    
    xio_device_node = Node(
        package="xio_device",
        executable="XioDeviceNode",
        # arguments=['--ros-args', '--log-level', 'debug'],
    )
    
    motor_zero_speed = launch.actions.ExecuteProcess(
        cmd=['ros2', 'service', 'call', '/set_motor_speed', 'thetis_interfaces/msg/SetFloat64', '"{data: 0.0}"']
    )

    ld.add_action(recorder_initialize)
    ld.add_action(recorder_pause)
    ld.add_action(motor_controller_node)
    ld.add_action(master_node)
    ld.add_action(xio_device_node)
    ld.add_action(motor_zero_speed)

    # TODO: Send service call to set motor speed to 0
    
    return ld