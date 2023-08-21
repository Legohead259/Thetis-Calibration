"""
ROS2 node for running the Waveshare DRV8825 HAT for the Raspberry Pi

CONOPS: The DRV8825 controller directly interfaces with the Raspberry Pi hardware 
attached on top (HAT) to drive a stepper motor. The controller can be commanded to 
drive the motor at a specific angular velocity or step-by-step, depending on the
end use case.

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

import rclpy
import ximu3
from rclpy.node import Node
from std_srvs.srv import Trigger
from thetis_interfaces.srv import XioCmd
from thetis_interfaces.msg import Inertial

class CalibrationMasterNode(Node):
    def __init__(self):
        super().__init__("CalibrationMasterNode")
        
        # Declare parameters
        
        # Get parameter values from node launch
        
        # Report parameter values
        
        # Create publishers
        
        # Create subscribers
        
        # Create services
        self.estop_service = self.create_service(Trigger, 'estop', self.estop_callback)
        
        # Create clients
        self.stop_motor_client = self.create_client(Trigger, "stop_motor")
        while not self.stop_motor_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("'stop_motor' service not available, waiting again...")
        self.stop_motor_request = Trigger.Request()
    
    
    # =================
    # === CALLBACKS ===
    # =================
    
    
    def estop_callback(self, request, response):
        """_summary_

        Args:
            request (_type_): _description_
            response (_type_): _description_
        """
        pass
    

def main(args=None):
    rclpy.init(args=args)
    node = CalibrationMasterNode()
    rclpy.spin(node)
    rclpy.shutdown()
    
if __name__ == "__main__":
    main()