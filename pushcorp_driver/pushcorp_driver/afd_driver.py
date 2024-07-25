import rclpy
from rclpy.node import Node
from pushcorp_msgs.msg import Value
import socket
import time 
import json


POSITION_ENDPOINT = "/afd/actualPosition"
FORCE_ENDPOINT = "/afd/actualForce"


class AFDDriver(Node):
    def __init__(self):
        super().__init__('afd_position_publisher')
        
        self.declare_parameter('ip')
        self.declare_parameter('port')
        self.declare_parameter('period')

        # Get the parameter values
        self.ip = self.get_parameter('ip').get_parameter_value().string_value
        self.port = self.get_parameter('port').get_parameter_value().integer_value
        period = self.get_parameter('period').get_parameter_value().double_value

        # Create the ROS2 interfaces
        self.pos_pub = self.create_publisher(Value, 'afd/position', 10)
        self.force_pub = self.create_publisher(Value, 'afd/force', 10)

        # Create a timer for reading from the AFD
        self.timer = self.create_timer(period, self.publish_udp_data)

    def read(self,
             udp_socket: socket.socket,
             endpoint: str) -> Value:
        udp_socket.sendto(endpoint.encode(), (self.ip, self.port))
        response, _ = udp_socket.recvfrom(1024)  # Adjust the buffer size as needed

        # Convert to message
        msg = Value()
        msg.value = json.loads(response.decode())['data'][endpoint]

        msg.timestamp = self.get_clock().now().to_msg()

        return msg

    def publish_udp_data(self): 
        # Socket communication
        udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        pos_msg = self.read(udp_socket, POSITION_ENDPOINT)
        force_msg = self.read(udp_socket, FORCE_ENDPOINT)
        udp_socket.close()

        # Publish the ROS message
        self.pos_pub.publish(pos_msg)
        self.force_pub.publish(force_msg)


def main(args=None):
    rclpy.init(args=args)
    afd_driver = AFDDriver()
    afd_driver.get_logger().info('Started AFD driver')
    rclpy.spin(afd_driver)
    afd_driver.destroy_node()
    rclpy.shutdown()
    
    
if __name__ == '__main__':
    main()


