import time
import math
from smbus2 import SMBus
from mpu6050 import mpu6050
from adafruit_servokit import ServoKit

# --- Setup ---
mpu = mpu6050(0x68)
kit = ServoKit(channels=16)

servo_pitch = kit.servo[1]
servo_roll  = kit.servo[0]

# Center servos
servo_pitch.angle = 90
servo_roll.angle  = 90

# Filter constants
alpha = 0.98  # gyro weight for complementary filter
dt = 0.02     # loop time

# --- Calibration ---
print("Calibrating... keep platform steady for 3 seconds")
time.sleep(3)

# Retry helper for I2C reads
def safe_read(func, retries=5, delay=0.01):
    for _ in range(retries):
        try:
            return func()
        except OSError:
            time.sleep(delay)
    raise OSError("I2C read failed after multiple retries")

base_acc = safe_read(mpu.get_accel_data)
base_gyro = safe_read(mpu.get_gyro_data)

# Baseline angles
pitch_base = math.degrees(math.atan2(base_acc['y'], base_acc['z']))
roll_base  = math.degrees(math.atan2(-base_acc['x'], base_acc['z']))

# --- Initialize angles ---
pitch = 0.0
roll  = 0.0

def map_range(x, in_min, in_max, out_min, out_max):
    return max(min(out_max, (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min), out_min)

print("Starting gimbal loop...")
while True:
    # --- Safe sensor reads ---
    accel = safe_read(mpu.get_accel_data)
    gyro  = safe_read(mpu.get_gyro_data)

    # Accelerometer angles
    pitch_acc = math.degrees(math.atan2(accel['y'], accel['z'])) - pitch_base
    roll_acc  = math.degrees(math.atan2(-accel['x'], accel['z'])) - roll_base

    # Gyro rates (deg/s)
    gx = gyro['x'] - base_gyro['x']
    gy = gyro['y'] - base_gyro['y']

    # Integrate gyro (angle change)
    pitch_gyro = pitch + gx * dt
    roll_gyro  = roll + gy * dt

    # Complementary filter: fuse accel + gyro
    pitch = alpha * pitch_gyro + (1 - alpha) * pitch_acc
    roll  = alpha * roll_gyro  + (1 - alpha) * roll_acc

    # Map to servo angles (opposite direction to stabilize)
    pitch_servo = map_range(-pitch, -45, 45, 0, 180)
    roll_servo  = map_range(-roll, -45, 45, 0, 180)

    # Move servos
    servo_pitch.angle = 180 - pitch_servo
    servo_roll.angle  = roll_servo

    # Debug output
    print(f"Pitch: {pitch:.2f}° -> Servo: {pitch_servo:.1f} | Roll: {roll:.2f}° -> Servo: {roll_servo:.1f}")

    time.sleep(dt)
