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

def get_float_parameter(node: Node, name: str) -> float:
    """Wrapper for rclpy nodes to return a parameter that is expected to have an float value

    Args:
        node (Node): the ROS2 node to pull the parameter from
        name (str): The name of the parameter. MUST BE THE SAME AS WHAT WAS DECLARED

    Returns:
        float: The value of the supplied parameter as gotten from the specific node
    """
    return node.get_parameter(name).get_parameter_value().float_value