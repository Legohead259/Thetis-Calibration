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
from rclpy.node import Node
from rclpy.serialization import serialize_message
import rosbag2_py as rosbag
from std_srvs.srv import Trigger
from thetis_interfaces.srv import XioCmd, TestTrigger
from thetis_interfaces.msg import Inertial
from calibrator_common.common.service_client import ServiceNames, create_client
from calibrator_common.common.pub_sub import TopicNames

class CalibrationMasterNode(Node):
    def __init__(self):
        super().__init__("CalibrationMasterNode")
        
        # Declare parameters
        
        # Get parameter values from node launch
        
        # Report parameter values
        
        # Create publishers
        
        # Create subscribers
        self.subscription = self.create_subscription(Inertial, TopicNames.INERTIAL_MESSAGES.value, self.inertial_msg_callback, 10)
        self.subscription  # prevent unused variable warning

        # Create services
        self.start_test_service = self.create_service(TestTrigger, ServiceNames.START_TEST.value, self.start_test_callback)
        self.stop_test_service = self.create_service(Trigger, ServiceNames.STOP_TEST, self.stop_test_callback)
        self.estop_service = self.create_service(Trigger, ServiceNames.ESTOP, self.estop_callback)
        
        # Create clients
        self.start_motor_client, self.start_motor_request = create_client(self, Trigger, ServiceNames.START_MOTOR)
        self.stop_motor_client, self.stop_motor_request = create_client(self, Trigger, ServiceNames.STOP_MOTOR)
        
        # Create recorder
        # self.writer = rosbag.SequentialWriter()
        
        # storage_options = rosbag._storage.StorageOptions(
        #     uri='bags',
        #     storage_id='mcap')
        # converter_options = rosbag._storage.ConverterOptions('', '')
        # self.writer.open(storage_options, converter_options)

        # topic_info = rosbag._storage.TopicMetadata(
        #     name=TopicNames.INERTIAL_MESSAGES.value,
        #     type='thetis_interfaces/msg/Inertial',
        #     serialization_format='cdr')
        # self.writer.create_topic(topic_info)
    
    
    # =================
    # === CALLBACKS ===
    # =================
    
    
    def inertial_msg_callback(self, msg):
        # self.writer.write(
        #     TopicNames.INERTIAL_MESSAGES.value,
        #     serialize_message(msg),
        #     self.get_clock().now().nanoseconds
        # )
        pass

    def start_test_callback(self, request, response):
        pass
    
    def stop_test_callback(self, request, reponse):
        pass
            
    def estop_callback(self, request, response):
        pass
    

def main(args=None):
    rclpy.init(args=args)
    node = CalibrationMasterNode()
    rclpy.spin(node)
    rclpy.shutdown()
    
if __name__ == "__main__":
    main()