#Run this on your host PC.
#If you are getting an error such as "No module named 'rclpy', try launching Thonny from a terminal

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import socket
import threading

UDP_PORT = 8888 #Make sure your Pico is broadcasting on this port. If you have multpiple Picos running at once, make these values unique
LISTEN_IP = ''  #If you want you can specify the Pico IP here but you can also leave it wide open
node_name = 'sensor_data' #Change this as desired

class UDPSensorNode(Node):
    def __init__(self):
        super().__init__('udp_sensor_node')

        # Create publisher
        self.publisher_ = self.create_publisher(String, node_name, 10)

        # Start UDP listener in a separate thread
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((LISTEN_IP, UDP_PORT))
        self.get_logger().info(f"Listening for UDP packets on port {UDP_PORT}")

        udp_thread = threading.Thread(target=self.udp_listener, daemon=True)
        udp_thread.start()

    def udp_listener(self):
        while rclpy.ok():
            try:
                data, addr = self.sock.recvfrom(1024)
                message = data.decode().strip()
                self.get_logger().info(f"Received: {message}")
                msg = String()
                msg.data = message
                self.publisher_.publish(msg)
            except Exception as e:
                self.get_logger().error(f"UDP error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = UDPSensorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.sock.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
