from machine import Pin, I2C
import time

# Setup I2C on GP4 (SDA) and GP5 (SCL)
i2c = I2C(0, scl=Pin(5), sda=Pin(4), freq=400_000)

BNO055_ADDRS = [0x28, 0x29]
BNO055_OPR_MODE = 0x3D
BNO055_EULER_H_LSB = 0x1A  # Starting register for Euler angles (heading, roll, pitch)

# Set BNO055 to NDOF mode
def set_mode(addr):
    i2c.writeto_mem(addr, BNO055_OPR_MODE, b'\x0C')  # NDOF mode
    time.sleep(0.1)

# Read Euler angles (6 bytes total)
def read_euler(addr):
    data = i2c.readfrom_mem(addr, BNO055_EULER_H_LSB, 6)
    heading = int.from_bytes(data[0:2], 'little') / 16.0
    roll    = int.from_bytes(data[2:4], 'little') / 16.0
    pitch   = int.from_bytes(data[4:6], 'little') / 16.0
    return heading, roll, pitch

# Initialize both sensors
for addr in BNO055_ADDRS:
    print(f"Initializing BNO055 at 0x{addr:02X}")
    set_mode(addr)

# Read loop
while True:
    for addr in BNO055_ADDRS:
        try:
            h, r, p = read_euler(addr)
            print(f"[0x{addr:02X}] Heading: {h:.2f}, Roll: {r:.2f}, Pitch: {p:.2f}")
        except OSError as e:
            print(f"[0x{addr:02X}] Read error:", e)
    print("-" * 40)
    time.sleep(1)
