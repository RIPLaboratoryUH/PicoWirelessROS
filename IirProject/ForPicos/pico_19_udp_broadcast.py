import network
import socket
from picozero import pico_led
import machine
import rp2
import sys
import time
from machine import I2C, Pin

# ==== Wi-Fi Settings ====
ssid = 'riplab'
password = 'Aut0m@tion'
UDP_PORT = 8819
BROADCAST_IP = '192.168.1.255'
SEND_INTERVAL = 0.01  # seconds

# ==== I2C Setup ====
i2c = I2C(0, scl=Pin(17), sda=Pin(16), freq=400_000)
ADS1115_ADDR = 0x48
ADS1115_CONVERSION = 0x00
ADS1115_CONFIG = 0x01

# ADS1115 config for differential AIN3 - AIN0, 0.256V range, 128SPS
ADS1115_CONFIG_REG = bytearray([
    0xEF,  # 1110 1111: OS=1, MUX=011 (AIN3 - AIN0), PGA=111 (±0.256V), MODE=1
    0x83   # 1000 0011: DR=128SPS, Comparator disabled
])

# Create UDP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

def read_sensor():
    # Start single conversion
    i2c.writeto_mem(ADS1115_ADDR, ADS1115_CONFIG, ADS1115_CONFIG_REG)
    time.sleep_ms(10)  # Wait for conversion

    # Read conversion result (2 bytes)
    raw = i2c.readfrom_mem(ADS1115_ADDR, ADS1115_CONVERSION, 2)
    value = int.from_bytes(raw, 'big', True)
    voltage = value * 4.096 / 32768  # Convert to volts
    timestamp = time.ticks_us()
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
        msg = f"{timestamp}, {sensor_value:.5f}\n"
        sock.sendto(msg.encode(), (BROADCAST_IP, UDP_PORT))
        print("Sent:", msg.strip())
        time.sleep(SEND_INTERVAL)

except KeyboardInterrupt:
    print("Stopped.")
finally:
    sock.close()