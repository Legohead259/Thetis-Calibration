import RPi.GPIO as GPIO
import time
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from thetis_interfaces.srv import SetFloat64
from enum import Enum
from math import log2, ceil

class MicroSteps(Enum):
    FULL_STEP           = 1
    HALF_STEP           = 2
    QUARTER_STEP        = 4
    EIGHTH_STEP         = 8
    SIXTEENTH_STEP      = 16
    THIRTY_SECOND_STEP  = 32
    
class StepModes(Enum):
    HARDWARE = 0
    SOFTWARE = 1
    
class MotorDirection(Enum):
    COUNTERCLOCKWISE = 0
    CLOCKWISE = 1

class DRV8825():
    _is_enabled : bool = False  # If the motor controller is enabled or not
    _mode : StepModes           # The microstep control mode. 0 (false) for hardware, 1 (true) for software
    _motor_dir : MotorDirection # The motor direction. 0 (false) for counter-clockwise, 1 (true) for clockwise
    _micro_steps : MicroSteps   # The number of microsteps currently being used
    _steps_per_rev : int        # The number of steps required to make one full revolution
    
    def __init__(self, dir_pin: int, step_pin: int, enable_pin: int, mode_pins: tuple, step_deg: float=1.8):
        self._dir_pin = dir_pin
        self._step_pin = step_pin        
        self._enable_pin = enable_pin
        self._mode_pins = mode_pins
        self._step_deg = step_deg
        self._steps_per_rev = 360.0 / step_deg
        
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        GPIO.setup(self._dir_pin, GPIO.OUT)
        GPIO.setup(self._step_pin, GPIO.OUT)
        GPIO.setup(self._enable_pin, GPIO.OUT)
        GPIO.setup(self._mode_pins, GPIO.OUT)
        
        self._step_pwm = GPIO.PWM(self._step_pin, 1) # Create PWM channel at 1 Hz
        self.set_micro_step(StepModes.SOFTWARE, MicroSteps.FULL_STEP)
        
    def calculate_step_delay(self, velocity: float):
        return (self._step_deg * self._micro_steps.value) / velocity
    
    def calculate_step_frequency(self, velocity: float):
        return 1 / self.calculate_step_delay(velocity)
    
    def turn_steps(self, motor_dir: MotorDirection, steps: int, step_delay: float=0.0025):
        self.enable()
        self.set_motor_direction(motor_dir)
        
        if (steps == 0):
            return
            
        for i in range(steps):
            self.step(step_delay)
        
    def step(self, step_delay: float=0.0025):
        self.digital_write(self._step_pin, True)
        time.sleep(step_delay)
        self.digital_write(self._step_pin, False)
        time.sleep(step_delay)
            
    def set_motor_direction(self, motor_dir: MotorDirection):
        self._motor_dir = motor_dir
        self.digital_write(self._dir_pin, motor_dir)
        
    def set_micro_step(self, mode: StepModes, step_format: MicroSteps):
        """If the mode is set to software, this function will set the appropriate pins to
        set the microstep amount. Otherwise, the controller defaults to the DIP switches
        physically present on the hardware.

        Args:
            mode (StepMode): the microstep set mode. 0 (false) for hardware, 1 (true) for software.
            
            stepformat (MicroSteps): the step format used for the motor controller; found 
            within the MicroSteps enum
        """
        microstep = [(0, 0, 0), # FULL_STEP
                     (1, 0, 0), # HALF_STEP
                     (0, 1, 0), # QUARTER_STEP
                     (1, 1, 0), # EIGHTH_STEP 
                     (0, 0, 1), # SIXTEENTH_STEP
                     (1, 0, 1)] # THIRTY_SECOND_STEP
        self._mode = mode
        self._micro_steps = step_format
        
        if (mode.value):
            self.digital_write(self.mode_pins, microstep[ceil(log2(step_format.value))])
            
    def enable(self):
        self._is_enabled = True
        self.digital_write(self._enable_pin, True)
            
    def disable(self):
        self._is_enabled = False
        self.digital_write(self._enable_pin, False)
        
    def cleanup_gpio(self):
        GPIO.cleanup()
            
    def digital_write(self, pin, value):
        GPIO.output(pin, value)
        
            
class DRV8825Node(DRV8825, Node):
    target_velocity : float = 360
    
    def __init__(self):
        Node.__init__(self, 'DRV8825Node')

        self.declare_parameter('dir_pin', 13)
        self.declare_parameter('step_pin', 19)
        self.declare_parameter('enable_pin', 12)
        self.declare_parameter('mode_pins', (16, 17, 20))
        self.declare_parameter('step_mode', StepModes.SOFTWARE.value)
        
        self.dir_pin = self.get_parameter('dir_pin').get_parameter_value().integer_value
        self.step_pin = self.get_parameter('step_pin').get_parameter_value().integer_value
        self.enable_pin = self.get_parameter('enable_pin').get_parameter_value().integer_value
        self.mode_pins = self.get_parameter('mode_pins').get_parameter_value().integer_array_value
        self.mode_pins = list(self.mode_pins)
        self.step_mode = self.get_parameter('step_mode').get_parameter_value().bool_value
        		
        self.get_logger().info(f"Using direction pin: {self.dir_pin}")
        self.get_logger().info(f"Using step pin: {self.step_pin}")
        self.get_logger().info(f"Using enable pin: {self.enable_pin}")
        self.get_logger().info(f"Using mode pins: {self.mode_pins}")
        self.get_logger().info(f"Using step mode: {self.step_mode}")
        
        DRV8825.__init__(self,
                         dir_pin=self.dir_pin, 
                         step_pin=self.step_pin, 
                         enable_pin=self.enable_pin, 
                         mode_pins=self.mode_pins)
        
        # Create services
        self.start_service = self.create_service(Trigger, 'start_motor', self.enable_motor_callback)
        self.stop_service = self.create_service(Trigger, 'stop_motor', self.disable_motor_callback)
        self.set_speed_service = self.create_service(SetFloat64, 'set_motor_speed', self.set_speed_callback)
        self.get_logger().info("Initialized services")
        
    def motor_loop(self):
        pass
                
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
        self._step_pwm.ChangeFrequency(self.calculate_step_frequency(request.data))
        response.success = True
        response.message = f"Motor speed set to {request.data} deg/sec"
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