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

from typing import Any

from rclpy.node import Node
from rclpy.client import Client
from enum import Enum

# Service Name Constants
class ServiceNames(str, Enum):
    # System
    START_TEST  = "start_test"  # [calibrator_interfaces/srv/TestTrigger] Tells the system to start the test number passed in the service call. Returns success (bool) and response message
    STOP_TEST   = "stop_test"   # [std_srvs/srv/Trigger] Stops the currently running test and resets.
    ESTOP       = "estop"       # [std_srvs/srv/Trigger] Emergency stops the entire system.
    
    # xioAPI
    XIO_SEND_CMD = "xio_send_cmd"   # [calibrator_interfaces/srv/XioCmd] Sends a JSON command to an xioAPI-compatible device. Returns the response JSON message
    
    # Plate
    START_MOTOR         = "start_motor"         # [std_srvs/srv/Trigger] Starts the motor. Returns success (bool) and response message
    STOP_MOTOR          = "stop_motor"          # [std_srvs/srv/Trigger] Stops the motor. Returns success (bool) and response message
    SET_MOTOR_SPEED     = "set_motor_speed"     # [calibrator_interfaces/srv/SetFloat64] Sets the motor speed in deg/sec. Returns success (bool) and response message
    SET_MOTOR_DIR       = "set_motor_dir"       # [calibrator_interfaces/srv/SetBool] Sets the motor direction. True for clockwise, False for counter-clockwise. Returns success (bool) and message
    STEP                = "step"                # [std_srvs/srv/Empty] Increments the motor a single step
    ZERO_MAGNETOMETER   = "zero_magnetometer"   # [std_srvs/srv/Trigger] Resets the magnetometer bias to current average. Returns success (bool) and message
    HOME                = "home"                # [std_srvs/srv/Trigger] Resets the plate to the "home" position (magnet directly on top of magnetometer). Returns success (bool) and message
    ZERO_ENCODER        = "zero_encoder"        # [std_srvs/srv/Trigger] Sets the current encoder position to 0. Must be stationary. Returns success (bool) and message
    RESET_ENCODER       = "reset_encoder"       # [std_srvs/srv/Trigger] Soft restarts the encoder. Returns success (bool) and message
    
    # rosbag recorder
    IS_PAUSED       = "is_paused"       # [rosbag2_interfaces/srv/IsPaused] Returns whether recording is currently paused.
    PAUSE           = "pause"           # [rosbag2_interfaces/srv/Pause] Pauses recording.
    RESUME          = "resume"          # [rosbag2_interfaces/srv/Resume] Resume recording if paused.
    SPLIT_BAGFILE   = "split_bagfile"   # [rosbag2_interfaces/srv/SplitBagfile] Triggers a split to a new file, even if none of the configured split criteria have been met yet
    SNAPSHOT        = "snapshot"        # [rosbag2_interfaces/srv/Snapshot] Enabled if --snapshot-mode is specified. Takes no arguments, triggers a snapshot.


def create_client(node: Node, srv_type: Any, name: str, blocking: bool=True, timeout: float=1.0) -> tuple [Client, Any]:
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