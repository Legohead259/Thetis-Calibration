from luma.core.interface.serial import i2c
from luma.core.render import canvas
from luma.oled.device import ssd1327
from subprocess import check_output
import time

def get_ip():
	cmd = "hostname -I | cut -d\' \' -f1"
	return check_output(cmd, shell=True).decode("utf-8").strip()

# rev.1 users set port=0
# substitute spi(device=0, port=0) below if using that interface
# substitute bitbang_6800(RS=7, E=8, PINS=[25,24,23,27]) below if using that interface
serial = i2c(port=1, address=0x3C)

# substitute ssd1331(...) or sh1106(...) below if using that device
device = ssd1327(serial)

with canvas(device) as draw:
    draw.rectangle(device.bounding_box, outline="white", fill="black")
    draw.text((15, 25), "Device IP Address", fill="white")
    draw.text((30, 40), get_ip(), fill="white")
    
time.sleep(5)