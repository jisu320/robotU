import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial
import sys

class MotorControlNode(Node):
    def __init__(self):
        super().__init__('motor_control')
        self.serial_port = '/dev/ttyUSB0' # 시리얼 포트 경로
        self.serial_baud = 9600  
        self.serial_connection = None

        self.create_subscription(
            String,
            'direction',
            self.direction_callback,
            10
        )

    def setup_serial(self):
        try:
			# 시리얼 연결 설정
            self.serial_connection = serial.Serial(self.serial_port, self.serial_baud)
        except serial.SerialException as e:
            self.get_logger().error(f"Serial connection failed: {e}")
            sys.exit(1)

    
    def direction_callback(self, msg):
        direction_command = msg.data
        self.get_logger().info(f"Dir Command: {direction_command}")
        self.send_command(direction_command)

    def send_command(self, command):
        if self.serial_connection:
            # 시리얼 연결 통해 명령 전송
            self.serial_connection.write(command.encode())

def main(args=None):
    rclpy.init(args=args)
    motor_control_node = MotorControlNode()
    motor_control_node.setup_serial()
    rclpy.spin(motor_control_node)
    motor_control_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()