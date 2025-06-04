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
adc = ADS1115(i2c, address=72, gain=4)

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

connect()

try:
    while True:
        sensor_value, timestamp = read_sensor()
        msg = f"{timestamp}, {sensor_value:.8f}\n"
        sock.sendto(msg.encode(), (BROADCAST_IP, UDP_PORT))
        print("Sent:", msg.strip())
        time.sleep(SEND_INTERVAL)

except KeyboardInterrupt:
    print("Stopped.")
finally:
    sock.close()


