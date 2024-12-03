import serial
from vpython import box, vector, rate
import numpy as np

# Configure the serial port
serial_port = "COM7"  # Change this to your port (e.g., "/dev/ttyUSB0" for Linux)
baud_rate = 115200
ser = serial.Serial(serial_port, baud_rate, timeout=1)

# Create the box in VPython
scene = box(size=vector(2, 1, 0.5), color=vector(0.5, 0.7, 0.8))

# Function to apply roll, pitch, and yaw rotations
def apply_rpy_rotation(box, roll, pitch, yaw):
    # Convert degrees to radians
    roll = np.radians(roll)
    pitch = np.radians(pitch)
    yaw = np.radians(yaw)
    
    # Rotation matrices for roll, pitch, and yaw
    R_roll = np.array([
        [1, 0, 0],
        [0, np.cos(roll), -np.sin(roll)],
        [0, np.sin(roll), np.cos(roll)]
    ])
    R_pitch = np.array([
        [np.cos(pitch), 0, np.sin(pitch)],
        [0, 1, 0],
        [-np.sin(pitch), 0, np.cos(pitch)]
    ])
    R_yaw = np.array([
        [np.cos(yaw), -np.sin(yaw), 0],
        [np.sin(yaw), np.cos(yaw), 0],
        [0, 0, 1]
    ])
    
    # Combined rotation matrix: R = R_yaw * R_pitch * R_roll
    R = np.dot(R_yaw, np.dot(R_pitch, R_roll))

    # Extract new axis vectors for the box
    box.axis = vector(R[0, 0], R[1, 0], R[2, 0])  # Forward direction
    box.up = vector(R[0, 1], R[1, 1], R[2, 1])    # Up direction

# Main loop to read from serial and update the visualization
try:
    while True:
        rate(50)  # Limit the loop to 50 updates per second

        # Read data from the serial port
        if ser.in_waiting > 0:
            line = ser.readline().decode('utf-8').strip()
            try:
                # Parse the line into roll, pitch, and yaw
                roll, pitch, yaw = map(float, line.split(','))
                print(f"Roll: {roll}, Pitch: {pitch}, Yaw: {yaw}")

                # Update the orientation of the box
                apply_rpy_rotation(scene, roll, pitch, yaw)
            except ValueError:
                print("Invalid data received:", line)
except KeyboardInterrupt:
    print("Exiting visualization...")
    ser.close()
