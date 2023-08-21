"""
ROS2 library file for common functions for declaring, importing, and handling services and clients

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
from rclpy.client import Client

def create_client(node: Node, srv_type, name: str, blocking: bool=True, timeout: float=1.0) -> tuple [Client, Request]:
    """Wrapper for rclpy to create a client

    Args:
        node (Node): The node that will handle the client
        srv_type (Any): The service the client will execute to the server
        name (str): The name of the service
        blocking (bool, optional): If the client will wait for the service to become available. Defaults to True.
        timeout (float, optional): Time client will wait for the service to become available before checking again or moving on. Defaults to 1.0.

    Returns:
        Client: The rclpy client for the node to use
        Request: The request that the client will send to the service 
    """
    _cli = node.create_client(srv_type, name)
    if blocking:
        while not _cli.wait_for_service(timeout_sec=timeout):
            node.get_logger().info(f"'{name}' service not available. Is server alive? Waiting again...")
    else:
        if not _cli.wait_for_service(timeout_sec=timeout):
            node.get_logger().info(f"'{name}' service not available. Is server alive?")
    return (_cli, srv_type.Request())