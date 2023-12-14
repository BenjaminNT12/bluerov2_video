import rclpy
from rclpy.node import Node
from pymavlink import mavutil

class MavlinkNode(Node):
    def __init__(self):
        super().__init__('mavlink_node')

        # Conéctate al vehículo BlueROV2
        self.master = mavutil.mavlink_connection(device='udp:192.168.2.1:14550', baudrate=115200)

        # Crea un temporizador para leer periódicamente los mensajes de MAVLink
        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        # Lee cualquier mensaje nuevo de MAVLink
        msg = self.master.recv_match(blocking=False)

        if msg is not None:
            # Haz algo con el mensaje de MAVLink
            self.get_logger().info('Received MAVLink message: %s' % msg)

def main(args=None):
    rclpy.init(args=args)

    mavlink_node = MavlinkNode()

    rclpy.spin(mavlink_node)

    mavlink_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()