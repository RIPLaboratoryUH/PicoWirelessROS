import network
import socket
from picozero import pico_led
import machine
import rp2
import sys
import time
from machine import ADC

# ==== Wi-Fi Settings ====
ssid = 'riplab'
password = 'Aut0m@tion'
UDP_PORT = 8816
BROADCAST_IP = '192.168.1.255'
SEND_INTERVAL = 0.01  # seconds

#==== ADC Settngs for Pico W
adc = ADC(26)
# Reference voltage for ADC (typically 3.3V on the Pico)
VREF = 3.3
# 12-bit ADC -> values range from 0 to 4095
ADC_RESOLUTION = 4095


# Create UDP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

def read_voltage():
    raw_value = adc.read_u16()
    scaled_value = raw_value >> 4
    voltage = (scaled_value / ADC_RESOLUTION) * VREF
    timestamp_us = time.ticks_us()
    return voltage, timestamp_us

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
        voltage, timestamp = read_voltage()
        msg = f"{timestamp},{voltage:.4f}\n"
        sock.sendto(msg.encode(), (BROADCAST_IP, UDP_PORT))
        print("Sent:", msg.strip())
        time.sleep(SEND_INTERVAL)

except KeyboardInterrupt:
    print("Stopped.")
finally:
    sock.close()
