import network
import socket
from picozero import pico_led
import machine
import rp2
import sys
import time

ssid = 'riplab'
password = 'Aut0m@tion'
UDP_PORT = 8888
BROADCAST_IP = '192.168.1.210'
SEND_INTERVAL = 1

# Create UDP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

def read_sensor():
    #do something here
    value = time.ticks_ms()
    return value
    

def connect():
    #Connect to WLAN
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)
    wlan.connect(ssid, password)
    while wlan.isconnected() == False:
        if rp2.bootsel_button() == 1:
            sys.exit()
        print('Waiting for connection...')
        pico_led.on()
        sleep(0.5)
        pico_led.off()
        sleep(0.5)
    ip = wlan.ifconfig()[0]
    print(f'Connected on {ip}')
    pico_led.on()
    return ip

connect()

try:
    while True:
        sensor_value = read_sensor()
        msg = f"Sensor value: {sensor_value:.2f}\n"
        sock.sendto(msg.encode(), (BROADCAST_IP, UDP_PORT))
        print("Sent:", msg.strip())
        time.sleep(SEND_INTERVAL)
except KeyboardInterrupt:
    print("Stopped.")
finally:
    sock.close()




