import time
import numpy as np
from scipy.signal import find_peaks
import board
import adafruit_mlx90393

i2c = board.I2C()  # uses board.SCL and board.SDA
# i2c = board.STEMMA_I2C()  # For using the built-in STEMMA QT connector on a microcontroller
SENSOR = adafruit_mlx90393.MLX90393(i2c, address=0x18, gain=adafruit_mlx90393.GAIN_1X)

# Circular buffer class
class CircularBuffer:
    def __init__(self, size):
        self.size = size
        self._buffer = np.zeros(size)
        self.index = 0
    
    def append(self, value):
        self._buffer[self.index] = value
        self.index = (self.index + 1) % self.size
    
    @property
    def data(self):
        return self._buffer
    
    @property
    def size(self):
        return self._size
    
    @size.setter
    def size(self, value):
        self._size = value
    
    
# Peak detection function
def detect_peaks(circular_buffer, threshold=0.0):
    data = circular_buffer.data
    peaks, _ = find_peaks(data, height=threshold)
    return peaks


def detect_direction(buf):
    direction_changes = []

    for i in range(buf.size):
        current_data = buf.data[i]
        previous_data = buf.data[(i - 1) % buf.size]
        
        if current_data >= 0 and previous_data < 0:
            direction_changes.append("Positive to Negative")
        elif current_data < 0 and previous_data >= 0:
            direction_changes.append("Negative to Positive")
            
    print(direction_changes)

# Simulation
buffer_size = 50
threshold = 10.0

circular_buffer = CircularBuffer(buffer_size)

while True:
    # Simulate sensor reading (replace with actual sensor data)
    sensor_reading = np.array(SENSOR.read_data)[1]
    
    circular_buffer.append(sensor_reading)
    
    if circular_buffer.index == 0:
        detected_peaks = detect_peaks(circular_buffer, threshold=threshold)
        if detected_peaks.any():
            print("Detected peaks:", detected_peaks)
        detect_direction(circular_buffer)
    print(circular_buffer.data)
    
    time.sleep(0.01)  # Simulate reading interval
