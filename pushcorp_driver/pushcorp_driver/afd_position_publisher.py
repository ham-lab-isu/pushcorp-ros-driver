import rclpy
from rclpy.node import Node
from pushcorp_msgs.msg import Value
import socket
import time 
import json

request_string = "/afd/actualPosition"

class AFDPositionPublisher(Node):

    def __init__(self):
        super().__init__('afd_position_publisher')
        
        self.declare_parameter('ip')
        self.declare_parameter('port')
        
        self.ip = self.get_parameter('ip').get_parameter_value().string_value  # Get the 'ip' parameter
        self.port = self.get_parameter('port').get_parameter_value().integer_value  # Get the 'port' parameter

        self.publisher_ = self.create_publisher(Value, 'pushcorp/afd/actualPosition', 10) 
        self.timer = self.create_timer(0.1, self.publish_udp_data)

    def publish_udp_data(self): 
        udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        udp_socket.sendto(request_string.encode(), (self.ip, self.port))
        response, _ = udp_socket.recvfrom(1024)  # Adjust the buffer size as needed
        actual_position = response.decode()
        udp_socket.close()

        # Extract the numerical value from the UDP data
        position_value = json.loads(actual_position)['data']['/afd/actualPosition']

        # Create a ROS 2 message with the numerical value
        msg = Value()
        msg.value = position_value

        # Add a timestamp to the message
        msg.timestamp = self.get_clock().now().to_msg()

        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    afd_position_publisher = AFDPositionPublisher()
    rclpy.spin(afd_position_publisher)
    afd_position_publisher.destroy_node()
    rclpy.shutdown()
    
    
if __name__ == '__main__':
    main()


