import asyncio
import board
import busio
import adafruit_bno055
import matplotlib.pyplot as plt
import time
import pandas as pd



# Create I2C bus
i2c = busio.I2C(board.SCL_1, board.SDA_1)  # Using pin 27 (SCL) and pin 28 (SDA)
# Create the BNO055 object
bno = adafruit_bno055.BNO055_I2C(i2c)
# Optionally change the sensor mode to NDOF mode to read fused data
bno.mode = adafruit_bno055.NDOF_MODE

    # Data collection setup
data = []  # List to store the data
collect_time = 10  # Time in seconds to collect the data
start_time = time.time()

print("Collecting data for {} seconds...".format(collect_time))

# Collect data
while time.time() - start_time < collect_time:
    # Read various sensor data
    timestamp = time.time()
    accel = bno.acceleration
    mag = bno.magnetic
    gyro = bno.gyro
    euler = bno.euler
    quat = bno.quaternion
    temperature = bno.temperature
    
    # Append the data to the list
    data.append([timestamp, *accel, *mag, *gyro, *euler, *quat, temperature])

# Convert the data into a pandas DataFrame
columns = ['time_s', 'Accel_X', 'Accel_Y', 'Accel_Z', 'Mag_X', 'Mag_Y', 'Mag_Z', 'Gyro_X', 'Gyro_Y', 'Gyro_Z', 'Euler_Heading', 'Euler_Roll', 'Euler_Pitch', 'Quat_W', 'Quat_X', 'Quat_Y', 'Quat_Z', 'Temp_C']
df = pd.DataFrame(data, columns=columns)

# Save the data to a CSV file
csv_filename = 'sensor_data.csv'
df.to_csv(csv_filename, index=False)
