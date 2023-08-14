"""
ROS2 node for running the I2C rotary encoder QT from Adafruit
https://www.adafruit.com/product/4991

CONOPS: The rotary encoder gives the operator fine control over the
rotation of the calibration plate. Every click of the encoder will
correspond to 1.8 degrees of motion (a single step). Pressing the 
encoder button will "home" the plate, such that the calibration cube
is vertical. Pressing and holding the encoder button will re-home 
the plate.

CHANGELOG:
 - Version 1.0.0: Initial release
 - Version 1.0.1: Updated the motor service response call to be a `debug` level
"""

import board
from adafruit_seesaw import seesaw, rotaryio, digitalio
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger, Empty
from thetis_interfaces.srv import SetFloat64, SetBool

__author__      = "Braidan Duffy"
__copyright__   = "Copyright 2023"
__credits__     = "Braidan Duffy"
__license__     = "MIT"
__version__     = "1.0.0"
__maintainer__  = "Braidan Duffy"
__email__       = "bduffy2018@my.fit.edu"
__status__      = "Prototype"


class RotaryEncoderNode(Node):
    _PRODUCT_ID = 4991
    _BUTTON_PIN = 24
    
    _i2c : board.I2C
    _addr : int
    _device : seesaw.Seesaw
    _button : digitalio.DigitalIO
    _button_held : bool
    _encoder : rotaryio.IncrementalEncoder
    _last_position : int
    _set_motor_dir_future = None
    _step_future = None
    
    def __init__(self):
        super().__init__("RotaryEncoderNode")
        
        # Configure ROS node parameters
        self.declare_parameter("i2c_address", 0x36)
        
        self._addr = self.get_parameter("i2c_address").get_parameter_value().integer_value
        
        self.get_logger().info("Using Rotary Encoder QT with address: 0x{:02X}".format(self._addr))
        
        # Create the SeeSaw device
        self._i2c = board.I2C()
        self._device = seesaw.Seesaw(self._i2c, self._addr)
        
        # Check that it is the Rotary Encoder QT module
        product_version = (self._device.get_version() >> 16) & 0xFFFF
        if product_version != self._PRODUCT_ID:
            self.get_logger().fatal("Unknown device detected at address: 0x{:02x}!\nWrong firmware loaded?  Expected {:d}".format(self._addr, self._PRODUCT_ID))
            self.destroy_node()
        
        # Configure encoder button input
        self._device.pin_mode(self._BUTTON_PIN, self._device.INPUT_PULLUP)
        self._button = digitalio.DigitalIO(self._device, self._BUTTON_PIN)
        self._button_held = False
        
        # Configure encoder rotation input
        self._encoder = rotaryio.IncrementalEncoder(self._device)
        self._last_position = 0
        
        # Create clients
        # self.home_client = self.create_client(Trigger, "home")
        # while not self.home_client.wait_for_service(timeout_sec=1.0):
        #     self.get_logger().info("'home' service not available, waiting again...")
        # self.home_req = Trigger.Request()
        
        # self.calibrate_home_client = self.create_client(Trigger, "calibrate_home")
        # while not self.home_client.wait_for_service(timeout_sec=1.0):
        #     self.get_logger().info("'calibrate_home' service not available, waiting again...")
        # self.calibrate_home_req = Trigger.Request()
        
        self.set_motor_dir_client = self.create_client(SetBool, "set_motor_dir")
        while not self.set_motor_dir_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("'set_motor_dir' service not available, waiting again...")
        self.set_motor_dir_req = SetBool.Request()
        self.get_logger().info("Established client for the 'set_motor_dir' service")
        
        self.step_client = self.create_client(Empty, "step")
        while not self.step_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("'step' service not available, waiting again...")
        self.step_req = Empty.Request()
        self.get_logger().info("Established client for the 'step' service")
        
        # Create periodic poll timer
        period = 0.05 # s
        self.poll_timer = self.create_timer(period, self.poll)
        self.get_logger().info(f"Created a timer to poll the encoder at {1/period} Hz")
        
    def poll(self):
        # negate the position to make clockwise rotation positive
        position = -self._encoder.position

        if position != self._last_position:
            self.get_logger().debug("Position: {}".format(position)) # DEBUG
            if position < self._last_position: # Counter-clockwise rotation
                self.send_motor_dir_request(False)
            else:
                self.send_motor_dir_request(True)
            self.send_step_request()
            self._last_position = position

        if not self._button.value and not self._button_held:
            self._button_held = True
            print("Button pressed") # DEBUG

        if self._button.value and self._button_held:
            self._button_held = False
            print("Button released") # DEBUG
        
    def send_home_request(self):
        future = self.home_client.call_async(self.home_req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()
    
    def send_calibrate_home_request(self):
        future = self.calibrate_home_client.call_async(self.calibrate_home_req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()
    
    def send_motor_dir_request(self, dir: bool):
        self.set_motor_dir_req.data = dir
        if self._set_motor_dir_future is not None and not self._set_motor_dir_future.done():
            self._set_motor_dir_future.cancel()  # Cancel the future. The callback will be called with Future.result == None.
            self.get_logger().warn("'set_motor_dir' Service Future canceled. "
                                   "The Node took too long to process the service call."
                                   "Is the Service Server still alive?")
        self._set_motor_dir_future = self.set_motor_dir_client.call_async(self.set_motor_dir_req)
        self._set_motor_dir_future.add_done_callback(self.request_done_callback)
        
    def send_step_request(self):
        if self._step_future is not None and not self._step_future.done():
            self._step_future.cancel()  # Cancel the future. The callback will be called with Future.result == None.
            self.get_logger().warn("'step' Service Future canceled."
                                   "The Node took too long to process the service call."
                                   "Is the Service Server still alive?")
        self._step_future = self.step_client.call_async(self.step_req)
        # self.future.add_done_callback(self.request_done_callback)
        
    def request_done_callback(self, future):
        response = future.result()
        
        if response is not None:
            self.get_logger().debug(f"[{response.success}]: {response.message}")
        else:
            self.get_logger().warn("Response did not come through")
    
def main(args=None):
    rclpy.init(args=args)
    encoder_node = RotaryEncoderNode()
    rclpy.spin(encoder_node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
