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

    motor_controller_node = Node(
        package="drv8825_motor_controller",
        executable="drv8825",
    )

    rotary_enocoder_node = Node(
        package="calibrator_inputs",
        executable="rotary_encoder"
    )

    ld.add_action(motor_controller_node)
    ld.add_action(rotary_enocoder_node)

    return ld