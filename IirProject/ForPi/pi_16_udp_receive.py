import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import socket
import threading

# Customize this
node_name = 'motor_16_voltage'

# Set your UDP listening address and port
LISTEN_IP = ''  # Listen on all interfaces
UDP_PORT = 8816        # Adjust as needed

class UDPSensorNode(Node):
    def __init__(self):
        super().__init__('motor_16_voltage_node')

        # Create publisher
        self.publisher_ = self.create_publisher(String, node_name, 10)

        # Setup UDP socket
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((LISTEN_IP, UDP_PORT))
        self.get_logger().info(f"Listening for UDP packets on port {UDP_PORT}")

        # Start listening in a separate thread
        udp_thread = threading.Thread(target=self.udp_listener, daemon=True)
        udp_thread.start()

    def udp_listener(self):
        while rclpy.ok():
            try:
                data, addr = self.sock.recvfrom(1024)
                message = data.decode().strip()
                self.get_logger().info(f"Received: {message}")

                # Publish the raw CSV line as a String
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
