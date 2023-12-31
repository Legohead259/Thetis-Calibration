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
from rclpy.task import Future
from rclpy.action import ActionClient
from std_srvs.srv import Trigger, Empty
from calibrator_interfaces.srv import SetFloat64, SetBool
from calibrator_interfaces.msg import MagnetDetect
from calibrator_interfaces.action import Home
from DRV8825 import DRV8825, StepModes, MicroSteps
from AMT22 import AMT22
from calibrator_common.common.parameters import ParameterNames, get_integer_parameter, get_integer_array_parameter, get_boolean_parameter
from calibrator_common.common.service_client import ServiceNames, create_client
from calibrator_common.common.pub_sub import TopicNames

class CalibratorPlateNode(Node):
    _controller: DRV8825
    
    _magnet_detect_flag = False
    _magnet_detect_future: Future = None
    
    def __init__(self):
        super().__init__('CalibratorPlateNode')
        
        # ========================
        # == DECLARE PARAMETERS ==
        # ========================
        
        # DRV8825 Parameters
        self.declare_parameter(ParameterNames.DIR_PIN.value, 13)
        self.declare_parameter(ParameterNames.STEP_PIN.value, 19)
        self.declare_parameter(ParameterNames.ENABLE_PIN.value, 12)
        self.declare_parameter(ParameterNames.MODE_PINS.value, (16, 17, 20))
        self.declare_parameter(ParameterNames.STEP_MODE.value, StepModes.HARDWARE.value)
        self.declare_parameter(ParameterNames.MICRO_STEPS.value, MicroSteps.FULL_STEP.value)
        
        # ====================
        # == GET PARAMETERS ==
        # ====================
        
        dir_pin = get_integer_parameter(self, ParameterNames.DIR_PIN.value)
        step_pin = get_integer_parameter(self, ParameterNames.STEP_PIN.value)
        enable_pin = get_integer_parameter(self, ParameterNames.ENABLE_PIN.value)
        mode_pins = get_integer_array_parameter(self, ParameterNames.MODE_PINS.value)
        step_mode = get_boolean_parameter(self, ParameterNames.STEP_MODE.value)
        micro_steps = get_integer_parameter(self, ParameterNames.MICRO_STEPS.value)
        		
        self.get_logger().info(f"Using direction pin: {dir_pin}")
        self.get_logger().info(f"Using step pin: {step_pin}")
        self.get_logger().info(f"Using enable pin: {enable_pin}")
        self.get_logger().info(f"Using mode pins: {mode_pins}")
        self.get_logger().info(f"Using step mode: {step_mode}")
        self.get_logger().info(f"Using microsteps: {micro_steps}")
        
        # ==========================
        # == CREATE SUBSCRIPTIONS ==
        # ==========================
        
        self.magnet_detect_subscription = self.create_subscription(MagnetDetect, TopicNames.MAGNET_DETECT.value, self.magnet_detect_callback, 10)
        
        # =====================
        # == CREATE SERVICES ==
        # =====================
        
        self.start_motor_service = self.create_service(Trigger, ServiceNames.START_MOTOR.value, self.enable_motor_callback)
        self.stop_motor_service = self.create_service(Trigger, ServiceNames.STOP_MOTOR.value, self.disable_motor_callback)
        self.set_motor_speed_service = self.create_service(SetFloat64, ServiceNames.SET_MOTOR_SPEED.value, self.set_speed_callback)
        self.set_motor_dir_service = self.create_service(SetBool, ServiceNames.SET_MOTOR_DIR.value, self.set_motor_direction_callback)
        self.step_service = self.create_service(Empty, ServiceNames.STEP.value, self.step_callback)
        self.home_service = self.create_service(Trigger, ServiceNames.HOME.value, self.home_callback)
        
        self.get_logger().info("Initialized services")
        
        # ====================
        # == CREATE CLIENTS ==
        # ====================
        
        # TODO: Make encoder_zero client
        self.magnet_detect_client, self.magnet_detect_request = create_client(self, Trigger, "magnet_detect")
        self.magnet_detect_action_client = ActionClient(self, Home, "magnet_detect")
        
        # ========================================
        # == INITIALIZE NODE-SPECIFIC VARIABLES ==
        # ========================================
        
        self._controller = DRV8825(dir_pin, step_pin, enable_pin, mode_pins, micro_steps)
    
    
    # ==============================
    # === SUBSCRIPTION CALLBACKS ===
    # ==============================
    
    def magnet_detect_callback(self, msg: MagnetDetect):
        pass
        # self._magnet_detect_flag = True
        # self.get_logger().info(f"Magnet detected at: {msg.timestamp}")
        # if self._magnet_detect_future is not None and not self._magnet_detect_future.done():
        #     self._magnet_detect_future.set_result(True)  # Set the result to unblock future
        
    
    # =========================
    # === SERVICE CALLBACKS ===
    # =========================
    
                
    def enable_motor_callback(self, request, response):
        if not self._controller.is_enabled:
            self._controller.enable()
            self._controller.start()
            response.success = True
            response.message = "Motor started"
        else:
            response.success = False
            response.message = "Motor is already running"
        return response

    def disable_motor_callback(self, request, response):
        if self._controller.is_enabled:
            self._controller.disable()
            self._controller.stop()
            response.success = True
            response.message = "Motor stopped"
        else:
            response.success = False
            response.message = "Motor is not running"
        return response

    def set_speed_callback(self, request, response):
        if (request.data == 0):
            self._controller.stop()
        else:
            self._controller.start()
            self._controller.pwm_frequency = self._controller.calculate_step_frequency(request.data)
        response.success = True
        response.message = f"Motor speed set to {request.data} deg/sec"
        return response
    
    def set_motor_direction_callback(self, request, response):
        self._controller.set_motor_direction(request.data)
        response.success = True
        response.message = "Motor direction set to clockwise" if self._controller._motor_dir else "Motor direction set to counter-clockwise"
        return response
    
    def step_callback(self, request, response):
        self.step()
        return response
    
    def home_callback(self, request, response):
        if not self._controller.is_enabled:
            self._controller.enable()
        self._controller.start()
        self._controller.pwm_frequency = 120
        self.send_magnet_detect_request()
        return response
        
    def send_magnet_detect_goal(self):
        self.magnet_detect_action_client.wait_for_server()
        self._magnet_detect_future = self.magnet_detect_action_client.send_goal_async(Home())
        self._magnet_detect_future.add_done_callback(self.request_done_callback)
        
    
    def send_magnet_detect_request(self):
        if self._magnet_detect_future is not None and not self._magnet_detect_future.done():
            self._magnet_detect_future.cancel()  # Cancel the future. The callback will be called with Future.result == None.
            self.get_logger().warn("'detect_magnet' Service Future canceled. "
                                   "The Node took too long to process the service call."
                                   "Is the Service Server still alive?")
        self._magnet_detect_future = self.magnet_detect_client.call_async(self.magnet_detect_request)
        self._magnet_detect_future.add_done_callback(self.request_done_callback)
            
            
    # =========================
    # === UTILITY FUNCTIONS ===
    # =========================
    
    
    def request_done_callback(self, future):
        response = future.result()
        self.get_logger().info(f"{response}")
        
        if response is not None:
            self.get_logger().debug(f"[{response.success}]: {response.message}")
            self._controller.stop()
        else:
            self.get_logger().warn("Response did not come through")
            
    def safe(self):
        self._controller.disable()
        
    def exit(self):
        self.safe()
        self._controller.cleanup_gpio()

            
def main(args=None):
    try:
        rclpy.init(args=args)
        node = CalibratorPlateNode()
        rclpy.spin(node)
        rclpy.shutdown()
    finally:
        node.exit()
        node.destroy_node()


if __name__ == '__main__':
    main()