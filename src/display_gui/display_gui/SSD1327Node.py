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
from thetis_interfaces.srv import SetFloat64, SetBool
from luma.core.interface.serial import i2c
from luma.core.render import canvas
from luma.oled.device import ssd1327
from luma.core.error import DeviceAddressError, DeviceNotFoundError, DevicePermissionError
from PIL import ImageFont
from display_gui.gui_elements import Button

class SSD1327Node(Node):
    _bus : i2c
    _PORT : int
    _ADDR : int
    _device : ssd1327
    
    def __init__(self):
        super().__init__("SSD1327_Node")
        
        self.declare_parameter("port", 1)
        self.declare_parameter("address", 0x3C)
        
        self._PORT = self.get_parameter("port").get_parameter_value().integer_value
        self._ADDR = self.get_parameter("address").get_parameter_value().integer_value
        
        self.get_logger().info(f"Looking for device at address 0x{self._ADDR:02x} on I2C bus {self._PORT}")
        
        try:
            self._bus = i2c(port=self._PORT, address=self._ADDR)
        except (DeviceAddressError, DeviceNotFoundError, DevicePermissionError) as e:
            self.get_logger().fatal(e.with_traceback())
            while(True): pass # Block further code execution
        
        self._device = ssd1327(self._bus)
        
    def button_test(self):
        # font = ImageFont.truetype("DejaVuSans.ttf", 20, encoding="UTF-8")
        # unicode_checkmark = "▷"
        
        # with canvas(self._device) as draw:
        #     draw.text((10,10), unicode_checkmark, font=font, align="center", fill="white")
        #     draw.rectangle(draw.textbbox((10,10), unicode_checkmark, font=font, align="center"))
        button = Button.Button((10,10), 64, 32, text="▷ Start", font=ImageFont.truetype("DejaVuSans.ttf", 20, encoding="UTF-8"))
        with canvas(self._device) as draw:
            button.render(draw)
        
    def test(self):
        with canvas(self._device) as draw:
            draw.rectangle(self._device.bounding_box, outline="white", fill="black")
            draw.text((30, 40), "Hello World", fill="white")
        
        while True:
            pass
    
def main(args=None):
    rclpy.init(args=args)
    display_node = SSD1327Node()
    display_node.button_test()
    rclpy.spin(display_node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
