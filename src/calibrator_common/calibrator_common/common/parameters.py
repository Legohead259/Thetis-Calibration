"""
ROS2 library file for common functions for declaring, importing, and handling parameters

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

from rclpy.node import Node
from enum import Enum


class ParameterNames(str, Enum):
    # Calibrator Plate Node
    DIR_PIN             = "dir_pin"
    STEP_PIN            = "step_pin"
    ENABLE_PIN          = "enable_pin"
    MODE_PINS           = "mode_pins"
    STEP_MODE           = "step_mode"
    MICRO_STEPS         = "micro_steps"
    ENCODER_RESOLUTION  = "encoder_resolution"
    ENCODER_PORT        = "encoder_port"
    ENCODER_CS_PIN      = "encoder_cs_pin"
    ENCODER_BUS_SPEED   = "encoder_bus_speed"
    ENCODER_BUS_DELAY   = "encoder_bus_delay"
    
    # Master Node
    ROSBAG_UUID = "rosbag_uuid"
    ROSBAG_PATH = "rosbag_path"
    
    # MLX90393 Node
    MLX90393_I2C_ADDRESS = "i2c_address"
    MLX90393_GAIN = "gain"
    
    # XioDevice Node
    TARGET_UDP_ADDRESS  = "target_udp_address"
    UDP_SEND_PORT       = "udp_send_port"
    UDP_RECEIVE_PORT    = "udp_receive_port"


def get_string_parameter(node: Node, name: str) -> str:
    """Wrapper for RCLPY nodes to return a parameter that is expected to have a string value

    Args:
        node (Node): The ROS2 node to pull the parameter from
        name (str): The name of the parameter. MUST BE THE SAME AS WHAT WAS DECLARED!

    Returns:
        str: The value of the supplied parameter as gotten from the specific node
    """
    return node.get_parameter(name).get_parameter_value().string_value


def get_integer_parameter(node: Node, name: str) -> int:
    """Wrapper for rclpy nodes to return a parameter that is expected to have an integer value

    Args:
        node (Node): the ROS2 node to pull the parameter from
        name (str): The name of the parameter. MUST BE THE SAME AS WHAT WAS DECLARED

    Returns:
        int: The value of the supplied parameter as gotten from the specific node
    """
    return node.get_parameter(name).get_parameter_value().integer_value


def get_integer_array_parameter(node: Node, name: str) -> list[int]:
    """Wrapper for rclpy nodes to return a parameter that is expected to have an array of integer values

    Args:
        node (Node): the ROS2 node to pull the parameter from
        name (str): The name of the parameter. MUST BE THE SAME AS WHAT WAS DECLARED

    Returns:
        list: The values of the supplied parameter as gotten from the specific node
    """
    return list(node.get_parameter(name).get_parameter_value().integer_array_value)


def get_float_parameter(node: Node, name: str) -> float:
    """Wrapper for rclpy nodes to return a parameter that is expected to have an float value

    Args:
        node (Node): the ROS2 node to pull the parameter from
        name (str): The name of the parameter. MUST BE THE SAME AS WHAT WAS DECLARED

    Returns:
        float: The value of the supplied parameter as gotten from the specific node
    """
    return node.get_parameter(name).get_parameter_value().float_value


def get_float_array_parameter(node: Node, name: str) -> list[float]:
    """Wrapper for rclpy nodes to return a parameter that is expected to have an array of float values

    Args:
        node (Node): the ROS2 node to pull the parameter from
        name (str): The name of the parameter. MUST BE THE SAME AS WHAT WAS DECLARED

    Returns:
        list: The values of the supplied parameter as gotten from the specific node
    """
    return list(node.get_parameter(name).get_parameter_value().float_array_value)


def get_boolean_parameter(node: Node, name: str) -> bool:
    """Wrapper for rclpy nodes to return a parameter that is expected to have a boolean value

    Args:
        node (Node): the ROS2 node to pull the parameter from
        name (str): The name of the parameter. MUST BE THE SAME AS WHAT WAS DECLARED

    Returns:
        bool: The values of the supplied parameter as gotten from the specific node
    """
    return node.get_parameter(name).get_parameter_value().bool_value