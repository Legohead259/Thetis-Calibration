"""
ROS2 node for the MLX90393 QT breakout board from Adafruit
https://www.adafruit.com/product/4022

CONOPS: The MLX90393 is a high-range tri-axial magnetometer that can detect strong magnetic
fields, like those from a neodymium magnets. The sensor will develop a baseline reading on 
startup (or on command from a service call) and use that to "zero" out the readings. Note,
the magnet on the plate should be as far away as possible when these calibrations occur. As
the magnet rotates, it will encounter the sensor and causing the readings to sharply increase.
This node will analyze those readings and determine the direction of the rotation and the peak.
It will then make a service call when a peak is detected that the plate can use to determine
the actual RPM and direction to verify proper function.

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

import board
import rclpy
import time
import numpy as np
from rclpy.node import Node
import adafruit_mlx90393
from calibrator_common.common.parameters import ParameterNames, get_integer_parameter
from calibrator_common.common.pub_sub import TopicNames, PlateDirection
from calibrator_common.common.service_client import ServiceNames
from calibrator_interfaces.msg import Magnetic, MagnetDetect
from std_srvs.srv import Trigger


class MLX90393Node(Node):
    _i2c = board.I2C()
    _addr : int
    _sensor : adafruit_mlx90393.MLX90393
    _offsets = np.array([0, 0, 0])
    _last_reading = 0
    _last_peak_call_time = 0
    _peak_call_timeout = 0.1
    
    def __init__(self):
        super().__init__("MLX90393Node")
        
        # Get parameters
        self.declare_parameter(ParameterNames.MLX90393_I2C_ADDRESS.value, 0x18)
        self.declare_parameter(ParameterNames.MLX90393_GAIN.value, adafruit_mlx90393.GAIN_1X)
        
        self._addr = get_integer_parameter(self, ParameterNames.MLX90393_I2C_ADDRESS.value)
        self._gain = get_integer_parameter(self, ParameterNames.MLX90393_GAIN.value)
        
        self.get_logger().info(f"Looking for MLX90393 at address: {self._addr}")
        self.get_logger().info(f"Setting MLX90393 gain to: {self._gain}")
        
        # Create publishers
        self.plate_magnetometer_publisher = self.create_publisher(Magnetic, TopicNames.PLATE_MAGNETOMETER.value, 10)
        self.magnet_detect_publisher = self.create_publisher(MagnetDetect, TopicNames.MAGNET_DETECT.value, 10)
        
        # Create callback timers
        self.timer = self.create_timer(0.01, self.poll)
        
        # Create services
        self.zero_magnetometer_service = self.create_service(Trigger, ServiceNames.ZERO_MAGNETOMETER.value, self.zero_magnetometer_callback)
        
        self.get_logger().info(f"Awaiting zero magnetometer service requests to be sent to /{ServiceNames.ZERO_MAGNETOMETER}")

        # Create node-specific variables
        self._sensor = adafruit_mlx90393.MLX90393(self._i2c, address=self._addr, gain=self._gain)
        self.calculate_offsets()
        
        
    # =========================
    # === SENSING FUNCTIONS ===
    # =========================
    
    
    def calculate_offsets(self):
        start_time = time.monotonic()
        values = np.array([0, 0, 0])
        num_readings = 0
        
        while time.monotonic() - start_time < 0.5: # Average readings over half a second
            readings = np.array(self._sensor.read_data)
        
            values += readings
            
            num_readings += 1
            time.sleep(0.01) # Sample at 100 Hz
            
        self._offsets = np.round(values / num_readings).astype(int)
        self.get_logger().info(f"Set offsets to: {self._offsets}")
        
    
    # ========================
    # === CALLBACK METHODS ===
    # ========================
    
    
    def poll(self):
        bias_readings = (np.array(self._sensor.read_data) - self._offsets).astype(float)
        self.plate_magnetometer_publisher.publish(Magnetic(
            timestamp=time.monotonic_ns(),
            mag_x=bias_readings[0],
            mag_y=bias_readings[1],
            mag_z=bias_readings[2]
        ))
        
        if self.detect_peak(bias_readings):
            if not self._sent_peak_notice:
                self.magnet_detect_publisher.publish()
        
    def detect_direction(self, current_reading: int):
        if self._last_reading < 0 and current_reading >= 0:
            direction = PlateDirection.COUNTER_CLOCKWISE
        elif self._last_reading > 0 and current_reading <= 0:
            direction = PlateDirection.CLOCKWISE
        else:
            direction = None
        
        self._last_reading = current_reading
        
        if direction:
            self.get_logger().info(f"Detected y-axis zero crossing: {direction}")
        
        return direction
        
    def detect_peak(self, readings: tuple[int, int, int]):
        return np.sqrt(readings[0]**2 + readings[2]**2) >= 2000
        
    def zero_magnetometer_callback(self, request, response):
        self.calculate_offsets()
        response.success = True
        response.message = f"Set offsets to: {self._offsets}"
        return response
        


def main(args=None):
    rclpy.init(args=args)
    node = MLX90393Node()
    rclpy.spin(node)
    rclpy.shutdown()
    
    
if __name__ == '__main__':
    main()