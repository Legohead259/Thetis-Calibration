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
import serial
from thetis_interfaces.msg import StringMsg

class SerialBridge(Node):
    _PORT: str
    _BAUD_RATE: int
    _TIMEOUT: float
    _ser: serial
    _TOPICNAME_RECEIVE: str
    _TOPICNAME_TRANSMIT: str
    
    def __init__(self):
        super().__init__("pyserial_bridge")
        
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baud_rate', 9600)
        self.declare_parameter('timeout', 0.1)
        
        self._PORT = self.get_parameter('port').get_parameter_value().string_value
        self._BAUD_RATE = self.get_parameter('baud_rate').get_parameter_value().integer_value
        self._TIMEOUT = self.get_parameter('timeout').get_parameter_value().double_value
        
        self.get_logger().info(f"Using port: {self._PORT}")
        self.get_logger().info(f"At baud rate: {self._BAUD_RATE}")
        self.get_logger().info(f"With timeout: {self._TIMEOUT}")
        
        try: 
            self._ser = serial.Serial(port = self._PORT, baudrate=self._BAUD_RATE, timeout=self._TIMEOUT)
        except serial.SerialException as e:
            self.get_logger().fatal("Port unavailable!")
            raise e
        
        self._ser.reset_input_buffer()  # Clear input buffer
        self._ser.reset_output_buffer() # Clear output buffer
        
        self._TOPICNAME_RECEIVE = f"serial/{self._PORT}/receive"
        self._TOPICNAME_TRANSMIT = f"serial/{self._PORT}/transmit"
    
        self.publisher_ = self.create_publisher(StringMsg,
                                                self._TOPICNAME_RECEIVE,
                                                10)
        self.get_logger().info(f"Publishing received serial messages to: {self._TOPICNAME_RECEIVE}")
        
        self.subscriber_ = self.create_subscription(StringMsg,
                                                    self._TOPICNAME_TRANSMIT,
                                                    self.serial_send,
                                                    10)
        self.get_logger().info(f"Listening for serial messages to transmit at: {self._TOPICNAME_TRANSMIT}")
        
        # Create a timer to call the serial listener function at 10 Hz
        self.timer_listener = self.create_timer(0.1, self.serial_listener)
        self.get_logger().info("Created a timer to check for new serial messages every 100 ms")
        
    def serial_send(self, msg: StringMsg):
        self.get_logger().debug(f"Sending {msg.msg} over port {self._PORT}")
        self._ser.write(bytes(msg.msg, 'utf-8'))
        
    def serial_listener(self):
        try:
            line = self.ser.readline().decode('utf-8').rstrip()
            if not line:
                return
        except:
            line = str(self.ser.readline())
            if not line:
                return
        self.get_logger().debug(f"Received {line} from port {self._PORT}")
        self.publisher_.publish(StringMsg(line))
    