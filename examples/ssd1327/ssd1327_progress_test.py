from progress_bar import BarBase
from luma.core.interface.serial import i2c
from luma.core.render import canvas
from luma.oled.device import ssd1327
from time import sleep

# rev.1 users set port=0
# substitute spi(device=0, port=0) below if using that interface
# substitute bitbang_6800(RS=7, E=8, PINS=[25,24,23,27]) below if using that interface
serial = i2c(port=1, address=0x3C)

# substitute ssd1331(...) or sh1106(...) below if using that device
device = ssd1327(serial)

def animate_progress_bar():
    for i in range(0, 101, 10):                                 # Create 10 frames
        with canvas(device) as draw:                            # Draw a new frame
            draw.text((0, 10), 'Connecting to', fill="white")   # Add a text line
            draw.text((0, 20), 'network...', fill="white")      # Add a text line
            bar = BarBase(10, 40, device.width - 20, 10, draw)  # Create a progress bar object
            bar.percent = i                                     # Update its percent filled and render it
        sleep(0.5)                                              # Render frame at close of with-statement
    
while True:
    animate_progress_bar()