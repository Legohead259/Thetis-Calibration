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
from std_srvs.srv import Trigger, Empty
from calibrator_interfaces.srv import SetFloat64, SetBool
from DRV8825 import DRV8825, StepModes, MicroSteps
from calibrator_common.common.parameters import ParameterNames, get_integer_parameter, get_integer_array_parameter, get_boolean_parameter
from calibrator_common.common.service_client import ServiceNames

class DRV8825Node(DRV8825, Node):
    target_velocity : float = 360
    
    def __init__(self):
        Node.__init__(self, 'DRV8825Node')
        
        self.declare_parameter(ParameterNames.DIR_PIN.value, 13)
        self.declare_parameter(ParameterNames.STEP_PIN.value, 19)
        self.declare_parameter(ParameterNames.ENABLE_PIN.value, 12)
        self.declare_parameter(ParameterNames.MODE_PINS.value, (16, 17, 20))
        self.declare_parameter(ParameterNames.STEP_MODE.value, StepModes.HARDWARE.value)
        self.declare_parameter(ParameterNames.MICRO_STEPS.value, MicroSteps.FULL_STEP.value)
        
        dir_pin = get_integer_parameter(self, ParameterNames.DIR_PIN.value)
        step_pin = get_integer_parameter(self, ParameterNames.STEP_PIN.value)
        enable_pin = get_integer_parameter(self, ParameterNames.ENABLE_PIN.value)
        mode_pins = get_integer_array_parameter(self, ParameterNames.MODE_PINS.value)
        self._step_mode = get_boolean_parameter(self, ParameterNames.STEP_MODE.value)
        micro_steps = get_integer_parameter(self, ParameterNames.MICRO_STEPS.value)
        		
        self.get_logger().info(f"Using direction pin: {dir_pin}")
        self.get_logger().info(f"Using step pin: {step_pin}")
        self.get_logger().info(f"Using enable pin: {enable_pin}")
        self.get_logger().info(f"Using mode pins: {mode_pins}")
        self.get_logger().info(f"Using step mode: {self._step_mode}")
        self.get_logger().info(f"Using microsteps: {micro_steps}")
        
        DRV8825.__init__(self,
                         dir_pin,
                         step_pin,
                         enable_pin,
                         mode_pins,
                         micro_steps)
        
        # Create services
        self.start_motor_service = self.create_service(Trigger, ServiceNames.START_MOTOR.value, self.enable_motor_callback)
        self.stop_motor_service = self.create_service(Trigger, ServiceNames.STOP_MOTOR.value, self.disable_motor_callback)
        self.set_motor_speed_service = self.create_service(SetFloat64, ServiceNames.SET_MOTOR_SPEED.value, self.set_speed_callback)
        self.set_motor_dir_service = self.create_service(SetBool, ServiceNames.SET_MOTOR_DIR.value, self.set_motor_direction_callback)
        self.step_service = self.create_service(Empty, ServiceNames.STEP.value, self.step_callback)
        
        self.get_logger().info("Initialized services")
                
    def enable_motor_callback(self, request, response):
        if not self._is_enabled:
            self.enable()
            self._step_pwm.start(50)
            response.success = True
            response.message = "Motor started"
        else:
            response.success = False
            response.message = "Motor is already running"
        return response

    def disable_motor_callback(self, request, response):
        if self._is_enabled:
            self.disable()
            self._step_pwm.stop()
            response.success = True
            response.message = "Motor stopped"
        else:
            response.success = False
            response.message = "Motor is not running"
        return response

    def set_speed_callback(self, request, response):
        if (request.data == 0):
            self._step_pwm.stop()
        else:
            self._step_pwm.start(50)
            self._step_pwm.ChangeFrequency(self.calculate_step_frequency(request.data))
        response.success = True
        response.message = f"Motor speed set to {request.data} deg/sec"
        return response
    
    def set_motor_direction_callback(self, request, response):
        self.set_motor_direction(request.data)
        response.success = True
        response.message = "Motor direction set to clockwise" if self._motor_dir else "Motor direction set to counter-clockwise"
        return response
    
    def step_callback(self, request, response):
        self.step()
        return response
                
            
def main(args=None):
    try:
        rclpy.init(args=args)
        stepper_node = DRV8825Node()
        rclpy.spin(stepper_node)
        rclpy.shutdown()
    finally:
        stepper_node.disable()
        stepper_node.cleanup_gpio()


if __name__ == '__main__':
    main()