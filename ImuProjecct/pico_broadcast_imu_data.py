import network
import socket
import time
from machine import Pin, I2C


# ==== USER CONFIG ====
SSID = 'riplab'
PASSWORD = 'Aut0m@tion'
DEST_IP = '192.168.1.255'  # Replace with your PC's IP
DEST_PORT = 8888

# ==== Wi-Fi Setup ====
wlan = network.WLAN(network.STA_IF)
wlan.active(True)
wlan.connect(SSID, PASSWORD)
while wlan.isconnected() == False:
    if rp2.bootsel_button() == 1:
        sys.exit()
    print('Waiting for connection...')
    time.sleep(0.5)
ip = wlan.ifconfig()[0]
print(f'Connected on {ip}')


# ==== UDP Socket Setup ====
udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# ==== I2C + BNO055 Setup ====
i2c = I2C(0, scl=Pin(5), sda=Pin(4), freq=400_000)

BNO055_ADDRS = [0x28, 0x29]
BNO055_OPR_MODE = 0x3D
BNO055_EULER_H_LSB = 0x1A  # Euler angle register start

def set_mode(addr):
    i2c.writeto_mem(addr, BNO055_OPR_MODE, b'\x0C')  # NDOF mode
    time.sleep(0.1)

def read_euler(addr):
    data = i2c.readfrom_mem(addr, BNO055_EULER_H_LSB, 6)
    heading = int.from_bytes(data[0:2], 'little') / 16.0
    roll    = int.from_bytes(data[2:4], 'little') / 16.0
    pitch   = int.from_bytes(data[4:6], 'little') / 16.0
    return heading, roll, pitch

for addr in BNO055_ADDRS:
    print(f"Initializing BNO055 at 0x{addr:02X}")
    set_mode(addr)

# ==== Main Loop ====
while True:
    payload = ''
    for addr in BNO055_ADDRS:
        try:
            h, r, p = read_euler(addr)
            sensor_id = 'A' if addr == 0x28 else 'B'
            line = f"{sensor_id}: Heading={h:.2f}, Roll={r:.2f}, Pitch={p:.2f}"
            print(line)
            payload += line + '\n'
        except OSError as e:
            err = f"{hex(addr)} read error: {e}"
            print(err)
            payload += err + '\n'

    # Send payload via UDP
    udp.sendto(payload.encode(), (DEST_IP, DEST_PORT))
    time.sleep(1)


