import RPi.GPIO as GPIO
import time
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from thetis_interfaces.srv import SetFloat64, SetBool
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
    
    def __init__(self, dir_pin: int, step_pin: int, enable_pin: int, mode_pins: tuple, micro_steps: MicroSteps, step_deg: float=1.8):
        self._dir_pin = dir_pin
        self._step_pin = step_pin        
        self._enable_pin = enable_pin
        self._mode_pins = mode_pins
        self._micro_steps = micro_steps
        self._step_deg = step_deg
        self._steps_per_rev = 360.0 / step_deg
        
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        GPIO.setup(self._dir_pin, GPIO.OUT)
        GPIO.setup(self._step_pin, GPIO.OUT)
        GPIO.setup(self._enable_pin, GPIO.OUT)
        GPIO.setup(self._mode_pins, GPIO.OUT)
        
        self._step_pwm = GPIO.PWM(self._step_pin, 1) # Create PWM channel at 1 Hz
        
    def calculate_step_delay(self, velocity: float) -> float:
        """Calculates the time between steps where the `step_pin` goes high then low again.
        This interval determines how fast the motor rotates.

        Args:
            velocity (float): the target angular velocity of the motor in degrees per second

        Returns:
            float: the time between the step pin going high the low again, in seconds
        """
        return (self._step_deg) / (2 * velocity * self._micro_steps)
    
    def calculate_step_frequency(self, velocity: float) -> float:
        """Calculates the number of steps will occur in a second given a target angular velocity
        This determines how fast the motor rotates.

        Args:
            velocity (float): the target angular velocity of the motor in degrees per second

        Returns:
            float: the total number of steps that will occur in a given second at the target velocity
        """
        return 1 / self.calculate_step_delay(velocity)
    
    def turn_steps(self, motor_dir: MotorDirection, steps: int, step_delay: float=0.0025):
        """Rotates the motor a given number of steps using the specified delay.
        Useful for moving something a set distance.

        Args:
            motor_dir (MotorDirection): the direction of the motor; 0 (false) for counter-clockwise, 1 (true) for clockwise
            steps (int): the number of steps to take
            step_delay (float, optional): the time between step cycles; this determines the motor speed. Defaults to 0.0025.
        """
        self.enable()
        self.set_motor_direction(motor_dir)
        
        if (steps == 0):
            return
            
        for i in range(steps):
            self.step(step_delay)
        
    def step(self, step_delay: float=0.0025):
        """Rotate the motor a single step

        Args:
            step_delay (float, optional): the time between step cycles. Defaults to 0.0025.
        """
        self.digital_write(self._step_pin, True)
        time.sleep(step_delay)
        self.digital_write(self._step_pin, False)
        time.sleep(step_delay)
            
    def set_motor_direction(self, motor_dir: MotorDirection):
        """Sets the motor direction in both hardware and in the internal state tracker

        Args:
            motor_dir (MotorDirection): the motor direction; 0 (false) for counter-clockwise, 1 (true) for clockwise
        """
        self._motor_dir = motor_dir
        self.digital_write(self._dir_pin, motor_dir)
        
    def set_micro_step(self, mode: StepModes, step_format: MicroSteps):
        """Sets the mode control pins to those appropriate for the different micro step modes.
        This mode does *not* supercede the physical mode switches present, therefore,
        in order for software control to work, each of the DIP switches must be set to 0 (low)

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
        """Enables the motor both physically and in the internal state
        """
        self._is_enabled = True
        self.digital_write(self._enable_pin, True)
            
    def disable(self):
        """Disables the motor both physically and in the internal state
        """
        self._is_enabled = False
        self.digital_write(self._enable_pin, False)
        
    def cleanup_gpio(self):
        """Resets all the GPIO to input and removes and pullup or pulldown settings
        """
        GPIO.cleanup()
            
    def digital_write(self, pin: int, value: bool):
        """Writes a digital value (true/false) to a given pin 

        Args:
            pin (int): the desired pin
            value (boolean): the digital state of the pin; 0 (false) for LOW, 1 (true) for HIGH
        """
        GPIO.output(pin, value)
        
    def digital_read(self, pin: int) -> bool:
        """Returns the digital state (true/false) of a given pin

        Args:
            pin (int): the desired pin

        Returns:
            bool: the digital state of the pin; 0 (false) for LOW, 1 (true) for HiGH
        """
        return GPIO.read(pin)
        
            
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
        self.start_service = self.create_service(Trigger, 'start_motor', self.enable_motor_callback)
        self.stop_service = self.create_service(Trigger, 'stop_motor', self.disable_motor_callback)
        self.set_speed_service = self.create_service(SetFloat64, 'set_motor_speed', self.set_speed_callback)
        self.set_dir_service = self.create_service(SetBool, 'set_motor_dir', self.set_motor_direction_callback)
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
        self._step_pwm.ChangeFrequency(self.calculate_step_frequency(request.data))
        response.success = True
        response.message = f"Motor speed set to {request.data} deg/sec"
        return response
    
    def set_motor_direction_callback(self, request, response):
        self.set_motor_direction(request.data)
        response.success = True
        response.message = "Motor direction set to clockwise" if self._motor_dir else "Motor direction set to counter-clockwise"
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