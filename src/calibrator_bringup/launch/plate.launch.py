"""
ROS2 launch file for the calibrator plate nodes

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
    return LaunchDescription([      
        # Spin up calibrator plate node
        Node(
            package="calibrator_plate",
            executable="plate"
        ),
        
        # Spin up magnetometer node
        Node(
            package="calibrator_plate",
            executable="magnetometer"
        ),
        
        # Spin up encoder node
        Node(
            package="calibrator_plate",
            executable="encoder"
        ),
        
        # Set motor speed to zero
        ExecuteProcess(
            cmd=['ros2', 'service', 'call', '/set_motor_speed', 'calibrator_interfaces/SetFloat64', '{data: 0.0}']
        ),
        
        # Start motor
        # ExecuteProcess(
        #     cmd=['ros2', 'service', 'call', '/start_motor', 'std_srvs/Trigger']
        # )
    ])