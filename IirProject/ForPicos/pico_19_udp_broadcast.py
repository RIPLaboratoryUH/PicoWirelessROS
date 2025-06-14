import network
import socket
from picozero import pico_led
import machine
import rp2
import sys
import time
from machine import I2C, Pin
from ads1x15 import ADS1115 

# ==== Wi-Fi Settings ====
ssid = 'riplab'
password = 'Aut0m@tion'
UDP_PORT = 8819
BROADCAST_IP = '192.168.1.255'
SEND_INTERVAL = 0.01  # seconds

# ==== I2C & ADC Setup ====
i2c = I2C(0, scl=Pin(17), sda=Pin(16), freq=400000)
adc = ADS1115(i2c, address=72, gain=5)

# Create UDP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

def read_sensor():
    timestamp = time.ticks_us()
    raw = adc.read(0, 2, 3)
    voltage = adc.raw_to_v(raw)
    return voltage, timestamp

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

connect()

try:
    while True:
        sensor_value, timestamp = read_sensor()
        if sensor_value >= 0.0001:
            msg = f"{timestamp}, {sensor_value:.8f}\n"
            sock.sendto(msg.encode(), (BROADCAST_IP, UDP_PORT))
            print("Sent:", msg.strip())
            log_data("log.txt", timestamp, sensor_value)
        time.sleep(SEND_INTERVAL)

except KeyboardInterrupt:
    print("Stopped.")
finally:
    sock.close()




