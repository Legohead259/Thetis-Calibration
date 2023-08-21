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
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    motor_controller_node = Node(package="drv8825_motor_controller",
                                 executable="drv8825")

    master_node = Node(package="calibration_master",
                       executable="master")
    
    xio_device_node = Node(package="xio_device",
                           executable="XioDeviceNode",
                        #    arguments=['--ros-args', '--log-level', 'debug'],
                           )

    ld.add_action(motor_controller_node)
    ld.add_action(master_node)
    ld.add_action(xio_device_node)

    return ld