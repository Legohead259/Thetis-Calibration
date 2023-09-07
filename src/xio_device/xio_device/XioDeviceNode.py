"""
ROS2 node for running interacting with inertial measurement devices that conform to the xioAPI
standard. API specification found here: https://x-io.co.uk/downloads/x-IMU3-User-Manual-v1.3.pdf

CONOPS: The IMU will be wirelessly connected over a UDP socket stream. The node can receive
telemetry over the `UDP_SEND` port defined in the xio device settings. These will then be
published to the `/measurement` topic for logging in the calibration master node. The node
will also run a server, `/xio_send_cmd`, that will take a message from a client that is the
**already formatted** xioAPI command string and send it to the xio device. The server will
then return the success status of the command and the response from the xio device (consult)
the API specification for what specific responses are expected. 

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
from rclpy.task import Future
from std_srvs.srv import Trigger
from calibrator_interfaces.srv import XioCmd
from calibrator_interfaces.msg import Inertial
from calibrator_common.common.parameters import ParameterNames, get_string_parameter, get_integer_parameter
from calibrator_common.common.pub_sub import TopicNames
from calibrator_common.common.service_client import ServiceNames, create_client
import time


class XioDeviceNode(Node):
    """ROS2 node for connecting to devices that use the xioAPI for data and commands.
    RE: the x-IMU3 and Thetis instrumentation board"""
    
    _connection_info : ximu3.UdpConnectionInfo
    _connection : ximu3.Connection
    
    _stop_motor_future : Future = None
    _estop_future : Future = None
    
    def __init__(self):
        super().__init__("XioDeviceNode")
                
        # Declare parameters
        self.declare_parameter(ParameterNames.TARGET_UDP_ADDRESS.value, "192.168.1.1")
        self.declare_parameter(ParameterNames.UDP_SEND_PORT.value, 9000)
        self.declare_parameter(ParameterNames.UDP_RECEIVE_PORT.value, 8000)
        
        # Get parameter values from node launch
        self._target_udp_address = get_string_parameter(self, ParameterNames.TARGET_UDP_ADDRESS)
        self._udp_send_port = get_integer_parameter(self, ParameterNames.UDP_SEND_PORT)
        self._udp_receive_port = get_integer_parameter(self, ParameterNames.UDP_RECEIVE_PORT)
        
        self.get_logger().info(f"Looking for device with IP address: {self._target_udp_address}")
        self.get_logger().info(f"Sending data to device through port: {self._udp_send_port}")
        self.get_logger().info(f"Listening for data from device through port: {self._udp_receive_port}")
        
        # Create publishers
        self.inertial_measurements_publisher = self.create_publisher(Inertial, TopicNames.INERTIAL.value, 10)
        
        self.get_logger().info(f"Sending inertial measurement data to topic: {self.inertial_measurements_publisher.topic_name}")
        
        # Create services
        self.xio_send_cmd_service = self.create_service(XioCmd, ServiceNames.XIO_SEND_CMD.value, self.xio_send_cmd_callback)
        
        self.get_logger().info(f"Awaiting xioAPI command service requests to be sent to /{ServiceNames.XIO_SEND_CMD}")
        
        # Create clients
        self.stop_motor_client, self.stop_motor_request = create_client(self, Trigger, ServiceNames.STOP_MOTOR.value)
        self.estop_client, self.estop_request = create_client(self, Trigger, ServiceNames.ESTOP.value)
        
        self.get_logger().info("Established clients")
        
        # Create node-specific variables
        self._connection_info = ximu3.UdpConnectionInfo(self._target_udp_address, self._udp_send_port, self._udp_receive_port)
        self._connection = ximu3.Connection(self._connection_info)
        while self._connection.open() != ximu3.RESULT_OK:
            self.get_logger().fatal(f"Unable to open connection. Is the device connected to the network? Waiting...", once=True)
            time.sleep(1)
        self.get_logger().info("Established connection to device!")
        
        # Set connection callbacks
        self.get_logger().debug("Attaching connection callbacks")
        self._connection.add_decode_error_callback(self.xio_decode_error_callback)
        self._connection.add_statistics_callback(self.xio_statistics_callback)
        self._connection.add_inertial_callback(self.xio_inertial_callback)
        
    def emergency_stop(self):
        self.send_stop_motor_request()
        self.send_estop_request()
    

    # ==============================
    # === CLIENT REQUEST METHODS ===
    # ==============================
    
    
    def send_stop_motor_request(self):
        if self._stop_motor_future is not None and not self._stop_motor_future.done():
            self._stop_motor_future.cancel()  # Cancel the future. The callback will be called with Future.result == None.
            self.get_logger().warn("'stop_motor' Service Future canceled. "
                                   "The Node took too long to process the service call."
                                   "Is the Service Server still alive?")
        self._stop_motor_future = self.stop_motor_client.call_async(self.stop_motor_request)
        self._stop_motor_future.add_done_callback(self.request_done_callback)
    
    def send_estop_request(self):
        if self._estop_future is not None and not self._estop_future.done():
            self._estop_future.cancel()  # Cancel the future. The callback will be called with Future.result == None.
            self.get_logger().warn("'estop' Service Future canceled. "
                                   "The Node took too long to process the service call."
                                   "Is the Service Server still alive?")
        self._estop_future = self.estop_client.call_async(self.estop_request)
        self._estop_future.add_done_callback(self.request_done_callback)
    
    
    # =================
    # === CALLBACKS ===
    # =================
    
    
    def xio_send_cmd_callback(self, request, reponse):
        pass
    
    def xio_decode_error_callback(self, decode_error):
        self.get_logger().error(ximu3.decode_error_to_string(decode_error))
        self.emergency_stop()
    
    def xio_statistics_callback(self, statistics):
        self.get_logger().debug(XioDeviceNode.timestamp_format(statistics.timestamp) +
                                XioDeviceNode.int_format(statistics.data_total) + " bytes" +
                                XioDeviceNode.int_format(statistics.data_rate) + " bytes/s" +
                                XioDeviceNode.int_format(statistics.message_total) + " messages" +
                                XioDeviceNode.int_format(statistics.message_rate) + " messages/s" +
                                XioDeviceNode.int_format(statistics.error_total) + " errors" +
                                XioDeviceNode.int_format(statistics.error_rate) + " errors/s")
    
    def xio_inertial_callback(self, message):
        self.get_logger().debug(XioDeviceNode.timestamp_format(message.timestamp) +
                                XioDeviceNode.float_format(message.gyroscope_x) + " deg/s" +
                                XioDeviceNode.float_format(message.gyroscope_y) + " deg/s" +
                                XioDeviceNode.float_format(message.gyroscope_z) + " deg/s" +
                                XioDeviceNode.float_format(message.accelerometer_x) + " g" +
                                XioDeviceNode.float_format(message.accelerometer_y) + " g" +
                                XioDeviceNode.float_format(message.accelerometer_z) + " g",
                                throttle_duration_sec=1)
        
        self.inertial_measurements_publisher.publish(
            Inertial(timestamp=message.timestamp,
                     accel_x=message.accelerometer_x,
                     accel_y=message.accelerometer_y,
                     accel_z=message.accelerometer_z,
                     gyro_x=message.gyroscope_x,
                     gyro_y=message.gyroscope_y,
                     gyro_z=message.gyroscope_z))
        
    def request_done_callback(self, future):
        response = future.result()
        
        if response is not None:
            self.get_logger().debug(f"[{response.success}]: {response.message}")
        else:
            self.get_logger().warn("Response did not come through")
    
    
    # ======================
    # === FORMAT HELPERS ===
    # ======================
    
    @staticmethod
    def timestamp_format(timestamp):
        return "{:8.0f}".format(timestamp) + " us"
    
    @staticmethod
    def int_format(value):
        return " " + "{:8.0f}".format(value)

    @staticmethod
    def float_format(value):
        return " " + "{:8.3f}".format(value)

    @staticmethod
    def string_format(string):
        return " \"" + string + "\""
        
        
def main(args=None):
    rclpy.init(args=args)
    node = XioDeviceNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
