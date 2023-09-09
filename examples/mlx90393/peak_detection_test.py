import time
import numpy as np
from scipy.signal import find_peaks
import board
import adafruit_mlx90393

i2c = board.I2C()  # uses board.SCL and board.SDA
# i2c = board.STEMMA_I2C()  # For using the built-in STEMMA QT connector on a microcontroller
SENSOR = adafruit_mlx90393.MLX90393(i2c, address=0x18, gain=adafruit_mlx90393.GAIN_1X)
sent_message = False

while True:
    # Simulate sensor reading (replace with actual sensor data)
    sensor_readings = np.array(SENSOR.read_data)
        
    if (np.linalg.norm(sensor_readings) > 2000):
        if not sent_message:
            print("Detected magnet above")
            sent_message = True
    else:
        sent_message = False
        
        
    time.sleep(0.01)  # Simulate reading interval
