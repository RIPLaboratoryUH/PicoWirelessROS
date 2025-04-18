# This code is meant to run on the host PC and is only for testing purposes. It simply recieves the UDP data and displays it. You can run this to verify that you are sending/recieving data without using ROS.

import socket

UDP_PORT = 8888  # Same port the Pico is sending to
LISTEN_IP = ''   # Empty string = listen on all interfaces

# Set up the UDP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((LISTEN_IP, UDP_PORT))

print(f"Listening for UDP packets on port {UDP_PORT}...")

try:
    while True:
        data, addr = sock.recvfrom(1024)  # buffer size is 1024 bytes
        print(f"Received from {addr}: {data.decode().strip()}")
except KeyboardInterrupt:
    print("\nStopped.")
finally:
    sock.close()
