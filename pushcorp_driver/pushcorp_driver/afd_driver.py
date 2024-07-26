import rclpy
from rclpy.node import Node
from pushcorp_msgs.msg import Value
from pushcorp_msgs.srv import SetControlMode, CommandValue
from std_srvs.srv import Trigger
import socket
import time 
import json


POSITION_ENDPOINT = "/afd/actualPosition"
FORCE_ENDPOINT = "/afd/actualForce"
COMMAND_FORCE_ENDPOINT = "/afd/commandForce"
COMMAND_POSITION_ENDPOINT = "/afd/commandPosition"
SET_CONTROL_MODE_ENDPOINT = "/afd/controlMode"
WEIGH_PAYLOAD_ENDPOINT = "/afd/weighPayload"


def create_socket():
  """ Helper for creating a UDP socket
  """
  return socket.socket(socket.AF_INET, socket.SOCK_DGRAM)


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
        self.weigh_payload_server = self.create_service(Trigger, 'weigh_payload', self.weigh_payload)
        self.set_control_mode_server = self.create_service(SetControlMode, 'set_control_mode', self.set_control_mode)
        self.command_force_server = self.create_service(CommandValue, 'command_force', self.command_force)
        self.command_position_server = self.create_service(CommandValue, 'command_position', self.command_position)

        # Create a timer for reading from the AFD
        self.timer = self.create_timer(period, self.publish_udp_data)

    def send(self,
             udp_socket: socket.socket,
             endpoint: str,
             buffer_size: int = 1024) -> dict:
      """ Sends data using a UDP socket
      """
      udp_socket.sendto(endpoint.encode(), (self.ip, self.port))
      response, _ = udp_socket.recvfrom(buffer_size)
      return json.loads(response.decode())

    def publish_udp_data(self): 
        def populate_value(udp_socket: socket.socket,
                           endpoint: str) -> Value:
            """ Helper function for evaluating a request into a ROS2 message
            """
            # Send the request
            response = self.send(udp_socket, endpoint)

            status = response['status']
            if status != 'success':
                self.get_logger().info(f'Failed to read endpoint \'{endpoint}\': {status}')
                return None

            # Convert to message
            msg = Value()
            msg.value = response['data'][endpoint]

            msg.timestamp = self.get_clock().now().to_msg()

            return msg

        # Socket communication
        udp_socket = create_socket()
        pos_msg = populate_value(udp_socket, POSITION_ENDPOINT)
        force_msg = populate_value(udp_socket, FORCE_ENDPOINT)
        udp_socket.close()

        # Publish the ROS message
        if pos_msg is not None:
            self.pos_pub.publish(pos_msg)

        if force_msg is not None:
            self.force_pub.publish(force_msg)

    def weigh_payload(self, _req: Trigger.Request, res: Trigger.Response) -> Trigger.Response:
        udp_socket = create_socket()
        response = self.send(udp_socket, WEIGH_PAYLOAD_ENDPOINT)
        res.success = response['status'] == 'success'
        if not res.success:
            res.message = 'Failed to weigh payload'

        return res

    def set_control_mode(self, req: SetControlMode.Request, res: SetControlMode.Response) -> SetControlMode.Response:
        udp_socket = create_socket()
        if not req.mode in (0, 1):
            res.success = False
            res.message = f'Mode must either be 0 (position) or 1 (force)'
            return res

        response = self.send(udp_socket, f'{SET_CONTROL_MODE_ENDPOINT}={req.mode}')
        res.success = response['status'] == 'success'
        if not res.success:
            res.message = 'Failed to set command mode'

        return res

    def command_force(self, req: CommandValue.Request, res: CommandValue.Response) -> CommandValue.Response:
        udp_socket = create_socket()
        response = self.send(udp_socket, f'{COMMAND_FORCE_ENDPOINT}={req.value}')
        res.success = response['status'] == 'success'
        if not res.success:
            res.message = 'Failed to command force'

        return res

    def command_position(self, req: CommandValue.Request, res: CommandValue.Response) -> CommandValue.Response:
        udp_socket = create_socket()
        response = self.send(udp_socket, f'{COMMAND_POSITION_ENDPOINT}={req.value}')
        res.success = response['status'] == 'success'
        if not res.success:
            res.message = 'Failed to command position'

        return res


def main(args=None):
    rclpy.init(args=args)
    afd_driver = AFDDriver()
    afd_driver.get_logger().info('Started AFD driver')
    rclpy.spin(afd_driver)
    afd_driver.destroy_node()
    rclpy.shutdown()
    
    
if __name__ == '__main__':
    main()


