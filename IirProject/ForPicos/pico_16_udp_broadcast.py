import network
import socket
from picozero import pico_led
import machine
import rp2
import sys
import time
from machine import ADC, I2C, Pin
from bno055 import BNO055
from ads1x15 import ADS1115

# ==== Wi-Fi Settings ====
ssid = 'riplab'
password = 'Aut0m@tion'
UDP_PORT = 8816
BROADCAST_IP = '192.168.1.255'
SEND_INTERVAL = 0.01  # seconds


# ==== I2C & ADC Setup ====
i2c0 = I2C(0, scl=Pin(17), sda=Pin(16), freq=400000)
adc = ADS1115(i2c0, address=72, gain=5)


# ==== Initialize IMU (BNO055 on I2C1, GP14=SDA, GP15=SCL) ====
i2c1 = I2C(1, scl=Pin(15), sda=Pin(14), freq=400000)
imu = BNO055(i2c1)

# ==== UDP Socket ====
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

def connect():
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)
    wlan.connect(ssid, password)
    while not wlan.isconnected():
        if rp2.bootsel_button():
            sys.exit()
        print('Waiting for connection...')
        pico_led.on()
        time.sleep(0.5)
        pico_led.off()
        time.sleep(0.5)
    ip = wlan.ifconfig()[0]
    print(f'Connected on {ip}')
    pico_led.on()
    return ip

def log_data(filename, *values):
    """
    Appends data to a log file. Values are comma-separated.
    
    Parameters:
        filename (str): Name of the file to log to (e.g., 'log.txt')
        *values: Any number of values to log
    """
    try:
        with open(filename, "a") as f:
            line = ",".join(str(v) for v in values) + "\n"
            f.write(line)
    except Exception as e:
        print("Logging error:", e)

def read_voltage():
    timestamp = time.ticks_us()
    raw = adc.read(0, 2, 3)
    voltage = adc.raw_to_v(raw)
    return voltage, timestamp

connect()

# ==== Main Loop ====
try:
    while True:
        voltage, timestamp = read_voltage()
        heading, roll, pitch = imu.read_imu()

        if heading is not None:
            msg = f"{timestamp},{voltage:.8f},{heading:.4f},{roll:.4f},{pitch:.4f}\n"
        else:
            msg = f"{timestamp},{voltage:.8f},NaN,NaN,NaN\n"

        sock.sendto(msg.encode(), (BROADCAST_IP, UDP_PORT))
        print("Sent:", msg.strip())
        if voltage >= 0.0001:
            log_data("log.txt", timestamp, voltage)

        time.sleep(SEND_INTERVAL)

except KeyboardInterrupt:
    print("Stopped.")
finally:
    sock.close()

