import RPi.GPIO as GPIO
import time
import rclpy
from rclpy.node import Node
from enum import Enum
import asyncio

class MicroSteps(Enum):
    FULL_STEP           = 0
    HALF_STEP           = 1
    QUARTER_STEP        = 2
    EIGHTH_STEP         = 3
    SIXTEENTH_STEP      = 4
    THIRTY_SECOND_STEP  = 5 

class DRV8825():
    _mode : bool # The microstep control mode. 0 (false) for hardware, 1 (true) for software
    _motor_dir : bool # The motor direction. 0 (false) for counter-clockwise, 1 (true) for clockwise
    
    def __init__(self, dir_pin: int, step_pin: int, enable_pin: int, mode_pins: tuple):
        self.dir_pin = dir_pin
        self.step_pin = step_pin        
        self.enable_pin = enable_pin
        self.mode_pins = mode_pins
        
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        GPIO.setup(self.dir_pin, GPIO.OUT)
        GPIO.setup(self.step_pin, GPIO.OUT)
        GPIO.setup(self.enable_pin, GPIO.OUT)
        GPIO.setup(self.mode_pins, GPIO.OUT)
    
    def set_micro_step(self, mode: bool, step_format: MicroSteps):
        """If the mode is set to software, this function will set the appropriate pins to
        set the microstep amount. Otherwise, the controller defaults to the DIP switches
        physically present on the hardware.

        Args:
            mode (bool): the microstep set mode. 0 (false) for hardware, 1 (true) for software.
            
            stepformat (MicroSteps): the step format used for the motor controller; found 
            within the MicroSteps enum
        """
        microstep = [(0, 0, 0), # FULL_STEP
                     (1, 0, 0), # HALF_STEP
                     (0, 1, 0), # QUARTER_STEP
                     (1, 1, 0), # EIGHTH_STEP 
                     (0, 0, 1), # SIXTEENTH_STEP
                     (1, 0, 1)] # THIRTY_SECOND_STEP
        if (mode):
            self.digital_write(self.mode_pins, microstep[step_format])
        
    async def turn_step(self, motor_dir: bool, steps: int, step_delay: float=0.0025):
        if (not motor_dir):
            self.digital_write(self.enable_pin, 1)
            self.digital_write(self.dir_pin, 0)
        else:
            self.digital_write(self.enable_pin, 1)
            self.digital_write(self.dir_pin, 1)

        if (steps == 0):
            return
            
        for i in range(steps):
            self.digital_write(self.step_pin, True)
            await asyncio.sleep(step_delay)
            self.digital_write(self.step_pin, False)
            await asyncio.sleep(step_delay)
            
    def stop(self):
        self.digital_write(self.enable_pin, 0)
            
    def digital_write(self, pin, value):
        GPIO.output(pin, value)
            
class DRV8825Node(DRV8825, Node):
    def __init__(self):
        Node.__init__(self, 'DRV8825Node')

        self.declare_parameter('dir_pin', 13)
        self.declare_parameter('step_pin', 19)
        self.declare_parameter('enable_pin', 12)
        self.declare_parameter('mode_pins', (16, 17, 20))
        
        self.dir_pin = self.get_parameter('dir_pin').get_parameter_value().integer_value
        self.step_pin = self.get_parameter('step_pin').get_parameter_value().integer_value
        self.enable_pin = self.get_parameter('enable_pin').get_parameter_value().integer_value
        self.mode_pins = self.get_parameter('mode_pins').get_parameter_value().integer_array_value
        self.mode_pins = list(self.mode_pins)
		
        self.get_logger().info(f"Using direction pin: {self.dir_pin}")
        self.get_logger().info(f"Using step pin: {self.step_pin}")
        self.get_logger().info(f"Using enable pin: {self.enable_pin}")
        self.get_logger().info(f"Using mode pins: {self.mode_pins}")
        
        DRV8825.__init__(self,
                         dir_pin=self.dir_pin, 
                         step_pin=self.step_pin, 
                         enable_pin=self.enable_pin, 
                         mode_pins=self.mode_pins)
            
def main(args=None):
    rclpy.init(args=args)

    drv8825 = DRV8825Node()
    
    drv8825.get_logger().info("Starting stepper motor movement...")
    asyncio.run(drv8825.turn_step(motor_dir=True, steps=100))

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    # drv8825.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()