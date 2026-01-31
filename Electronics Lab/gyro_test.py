from mpu6050 import mpu6050
import time
import math

sensor = mpu6050(0x68)

# Filter settings
dt = 0.01       # 100 Hz loop
alpha = 0.98    # complementary filter constant

# --- Initialize angles using accelerometer ---
accel = sensor.get_accel_data()
ax, ay, az = accel['x'], accel['y'], accel['z']
pitch_acc = math.degrees(math.atan2(ax, math.sqrt(ay*ay + az*az)))
roll_acc = math.degrees(math.atan2(ay, math.sqrt(ax*ax + az*az)))

pitch = pitch_acc
roll = roll_acc

# Gyro offsets (calibration to reduce drift)
gx_offset, gy_offset, gz_offset = 0, 0, 0
samples = 200
print("Calibrating gyroscope...")
for i in range(samples):
    g = sensor.get_gyro_data()
    gx_offset += g['x']
    gy_offset += g['y']
    gz_offset += g['z']
    time.sleep(0.005)
gx_offset /= samples
gy_offset /= samples
gz_offset /= samples
print("Calibration done.")

try:
    while True:
        # --- Read sensor data ---
        accel = sensor.get_accel_data()
        gyro = sensor.get_gyro_data()
        temp = sensor.get_temp()

        ax, ay, az = accel['x'], accel['y'], accel['z']
        gx, gy, gz = gyro['x'] - gx_offset, gyro['y'] - gy_offset, gyro['z'] - gz_offset

        # --- Accelerometer angles ---
        pitch_acc = math.degrees(math.atan2(ax, math.sqrt(ay*ay + az*az)))
        roll_acc  = math.degrees(math.atan2(ay, math.sqrt(ax*ax + az*az)))

        # --- Integrate gyro data ---
        pitch_gyro = pitch + gx * dt
        roll_gyro  = roll + gy * dt

        # --- Complementary filter ---
        pitch = alpha * pitch_gyro + (1 - alpha) * pitch_acc
        roll  = alpha * roll_gyro  + (1 - alpha) * roll_acc

        # --- Print results ---
        print(f"Pitch: {pitch:.2f}°, Roll: {roll:.2f}°")
        print(f"Gyro -> X: {gx:.2f}, Y: {gy:.2f}, Z: {gz:.2f}")
        print(f"Accel -> X: {ax:.2f}, Y: {ay:.2f}, Z: {az:.2f}")
        print(f"Temperature: {temp:.2f} °C")
        print("-" * 40)

        time.sleep(dt)

except KeyboardInterrupt:
    print("Closing...")
    time.sleep(1)
