"""
ROS2 node for running the AMT22 absolute encoder

CONOPS: 

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
from std_srvs.srv import Trigger, Empty
from calibrator_interfaces.msg import AngularVelocity, EncoderPosition, EncoderAngle
from calibrator_interfaces.srv import SetFloat64, SetBool
from AMT22 import AMT22
from calibrator_common.common.parameters import ParameterNames, get_integer_parameter, get_integer_array_parameter, get_boolean_parameter
from calibrator_common.common.service_client import ServiceNames
from calibrator_common.common.pub_sub import TopicNames
import time

class AMT22Node(Node):
    _sensor : AMT22
    
    def __init__(self):
        super().__init__('AMT22Node')
        
        # ========================
        # == DECLARE PARAMETERS ==
        # ========================
        
        self.declare_parameter(ParameterNames.ENCODER_RESOLUTION.value, 14)
        self.declare_parameter(ParameterNames.ENCODER_PORT.value, 0)
        self.declare_parameter(ParameterNames.ENCODER_CS_PIN.value, 0)
        self.declare_parameter(ParameterNames.ENCODER_BUS_SPEED.value, 500000)
        self.declare_parameter(ParameterNames.ENCODER_BUS_DELAY.value, 3)
        
        # ====================
        # == GET PARAMETERS ==
        # ====================
        
        encoder_resolution = get_integer_parameter(self, ParameterNames.ENCODER_RESOLUTION.value)
        encoder_port = get_integer_parameter(self, ParameterNames.ENCODER_PORT.value)
        encoder_cs_pin = get_integer_parameter(self, ParameterNames.ENCODER_CS_PIN.value)
        encoder_bus_speed = get_integer_parameter(self, ParameterNames.ENCODER_BUS_SPEED.value)
        encoder_bus_delay = get_integer_parameter(self, ParameterNames.ENCODER_BUS_DELAY.value)
        
        self.get_logger().info(f"Using encoder resolution: {encoder_resolution}")
        self.get_logger().info(f"Using SPI port: {encoder_port}")
        self.get_logger().info(f"Using CS pin: {encoder_cs_pin}")
        self.get_logger().info(f"Using SPI bus speed: {encoder_bus_speed} Hz")
        self.get_logger().info(f"Using SPI bus byte delay: {encoder_bus_delay} us")
        
        # ===========================
        # == INITIALIZE PUBLISHERS ==
        # ===========================
        
        self.angular_velocity_publisher_ = self.create_publisher(AngularVelocity, TopicNames.ENCODER_ANGULAR_VELOCITY.value, 10)
        self.angle_publisher_ = self.create_publisher(EncoderAngle, TopicNames.ENCODER_ANGLE.value, 10)
        self.position_publisher_ = self.create_publisher(EncoderPosition, TopicNames.ENCODER_POSITION.value, 10)
        
        self.angular_velocity_timer = self.create_timer(0.1, self.angular_velocity_callback)
        self.angle_timer = self.create_timer(0.1, self.angle_callback)
        self.position_timer = self.create_timer(0.1, self.position_callback)
        
        # ========================================
        # == INITIALIZE NODE-SPECIFIC VARIABLES ==
        # ========================================
        
        self._sensor = AMT22(encoder_resolution, encoder_port, encoder_cs_pin, encoder_bus_speed, encoder_bus_delay)
        
    
    # =======================
    # === TIMER CALLBACKS ===
    # =======================
    
    
    def angular_velocity_callback(self):
        pass
    
    def angle_callback(self):
        self.angle_publisher_.publish(EncoderAngle(
            timestamp=time.monotonic_ns(),
            angle=self._sensor.angle
        ))
    
    def position_callback(self):
        position, is_valid = self._sensor.position
        
        self.position_publisher_.publish(EncoderPosition(
            timestamp=time.monotonic_ns(),
            position=position,
            is_valid=is_valid
        ))
    
    
def main(args=None):
    rclpy.init(args=args)

    node = AMT22Node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
        
    
        