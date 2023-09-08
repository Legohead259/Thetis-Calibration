"""
DRV8825 Object for interfacing with the WaveShare dual motor HAT for Raspberry Pi

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

import RPi.GPIO as GPIO
import time
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
    
    
    # ==================
    # === PROPERTIES ===
    # ==================
    
    
    @property
    def is_enabled(self):
        """Returns the current internal enabled state
        """
        return self._is_enabled
    
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
        
    def start(self):
        self._step_pwm.start(50)
        
    def stop(self):
        self._step_pwm.stop()
    
    @property
    def micro_step(self):
        current_setting = self.digital_read(self._mode_pins)
        if   (0,0,0) == current_setting: return MicroSteps.FULL_STEP
        elif (1,0,0) == current_setting: return MicroSteps.HALF_STEP
        elif (0,1,0) == current_setting: return MicroSteps.QUARTER_STEP
        elif (1,1,0) == current_setting: return MicroSteps.EIGHTH_STEP
        elif (0,0,1) == current_setting: return MicroSteps.SIXTEENTH_STEP
        elif (1,0,1) == current_setting: return MicroSteps.THIRTY_SECOND_STEP
            
        
    @micro_step.setter
    def micro_step(self, val: tuple[StepModes, MicroSteps]):
        """Sets the mode control pins to those appropriate for the different micro step modes.
        This mode does *not* supercede the physical mode switches present, therefore,
        in order for software control to work, each of the DIP switches must be set to 0 (low)

        Args:
            mode (StepMode): the microstep set mode. 0 (false) for hardware, 1 (true) for software.
            
            step_format (MicroSteps): the step format used for the motor controller; found 
            within the MicroSteps enum
        """
        try:
            mode, step_format = val
        except ValueError:
            raise ValueError("Argument must be an iterable with StepMode and MicroSteps")
        else:
            microstep = [(0, 0, 0), # FULL_STEP
                        (1, 0, 0), # HALF_STEP
                        (0, 1, 0), # QUARTER_STEP
                        (1, 1, 0), # EIGHTH_STEP 
                        (0, 0, 1), # SIXTEENTH_STEP
                        (1, 0, 1)] # THIRTY_SECOND_STEP
            self._mode = mode
            self._micro_steps = step_format
            
            if (mode.value):
                self.digital_write(self._mode_pins, microstep[ceil(log2(step_format.value))])
                
    @property
    def pwm_frequency(self):
        pass
    
    @pwm_frequency.setter
    def pwm_frequency(self, value: int):
        self._step_pwm.ChangeFrequency(value)
    
    
    # =========================
    # === UTILITY FUNCTIONS ===
    # =========================
    
    
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
    
    def cleanup_gpio(self):
        """Resets all the GPIO to input and removes and pullup or pulldown settings
        """
        GPIO.cleanup()
        

if __name__ == "__main__":
    try:
        driver = DRV8825(13, 19, 12, (16, 17, 20), StepModes.HARDWARE.value)
        driver.enable()
        driver.turn_steps(True, 60000)
    finally:
        driver.disable()
        driver.cleanup_gpio()
    