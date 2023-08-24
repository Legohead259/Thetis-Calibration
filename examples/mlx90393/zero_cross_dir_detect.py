import time
import adafruit_mlx90393
import board

def detect_zero_crossing_with_direction(sensor, axis='y', threshold=0.0):
    last_reading = 0.0
    zero_crossings = 0
    
    while True:
        mag_x, mag_y, mag_z = sensor.magnetic
        
        if axis == 'y':
            current_reading = mag_y
        elif axis == 'x':
            current_reading = mag_x
        elif axis == 'z':
            current_reading = mag_z
        else:
            raise ValueError("Invalid axis. Choose 'x', 'y', or 'z'.")
        
        if last_reading < threshold and current_reading >= threshold:
            direction = 'Negative to Positive'
            zero_crossings += 1
        elif last_reading > threshold and current_reading <= threshold:
            direction = 'Positive to Negative'
            zero_crossings += 1
        else:
            direction = None
        
        last_reading = current_reading
        time.sleep(0.01)  # Adjust the delay between readings
        
        if direction:
            print(f"Detected {axis}-axis zero crossing: {direction}")
    
    return zero_crossings

i2c = board.I2C()  # Initialize I2C connection
sensor = adafruit_mlx90393.MLX90393(i2c, address=0x18)  # Initialize sensor

# Set the threshold for zero-crossing detection
threshold = 0.0  # Adjust this value based on your needs

axis_to_detect = 'y'
zero_crossings = detect_zero_crossing_with_direction(sensor, axis=axis_to_detect, threshold=threshold)
