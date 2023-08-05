"""
ROS2 node for interacting with the calibration controller (Adafruit RP2040 Macropad)

CONOPS: The controller will act as a human input device (HID) that sends a series of ASCII characters
to the calibration controller. Each sequence will correspond to a specific test and the calibrator
will react accordingly.

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

# Serial server node
import rclpy
from rclpy.node import Node

from SerialBridge import SerialBridge

class Controller(Node):
    def __init__(self):
        pass

def main(args=None):
	rclpy.init(args=args)
	serial_server = SerialBridge()
	rclpy.spin(serial_server)

if __name__ == '__main__':
	main()