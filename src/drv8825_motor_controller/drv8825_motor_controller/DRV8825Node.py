"""
ROS2 node for running the Waveshare DRV8825 HAT for the Raspberry Pi

CONOPS: The DRV8825 controller directly interfaces with the Raspberry Pi hardware 
attached on top (HAT) to drive a stepper motor. The controller can be commanded to 
drive the motor at a specific angular velocity or step-by-step, depending on the
end use case.

CHANGELOG:
 - Version 1.0.0: Initial release
"""

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger, Empty
from thetis_interfaces.srv import SetFloat64, SetBool
from DRV8825 import DRV8825, StepModes, MicroSteps

class DRV8825Node(DRV8825, Node):
    target_velocity : float = 360
    
    def __init__(self):
        Node.__init__(self, 'DRV8825Node')

        self.declare_parameter('dir_pin', 13)
        self.declare_parameter('step_pin', 19)
        self.declare_parameter('enable_pin', 12)
        self.declare_parameter('mode_pins', (16, 17, 20))
        self.declare_parameter('step_mode', StepModes.HARDWARE.value)
        self.declare_parameter('micro_steps', MicroSteps.THIRTY_SECOND_STEP.value)
        
        dir_pin = self.get_parameter('dir_pin').get_parameter_value().integer_value
        step_pin = self.get_parameter('step_pin').get_parameter_value().integer_value
        enable_pin = self.get_parameter('enable_pin').get_parameter_value().integer_value
        mode_pins = self.get_parameter('mode_pins').get_parameter_value().integer_array_value
        mode_pins = list(mode_pins)
        micro_steps = self.get_parameter('micro_steps').get_parameter_value().integer_value
        self._step_mode = self.get_parameter('step_mode').get_parameter_value().bool_value
        		
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
        self.start_motor_service = self.create_service(Trigger, 'start_motor', self.enable_motor_callback)
        self.stop_motor_service = self.create_service(Trigger, 'stop_motor', self.disable_motor_callback)
        self.set_motor_speed_service = self.create_service(SetFloat64, 'set_motor_speed', self.set_speed_callback)
        self.set_motor_dir_service = self.create_service(SetBool, 'set_motor_dir', self.set_motor_direction_callback)
        self.step_service = self.create_service(Empty, 'step', self.step_callback)
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