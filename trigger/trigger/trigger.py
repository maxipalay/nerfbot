import rclpy
from rclpy.node import Node
import serial
import serial.tools.list_ports as list_ports
from trigger_interfaces.srv import Fire

class Trigger(Node):
    def __init__(self):
        super().__init__("trigger")

        ARDUINO_SERIAL = "85439313130351612011"
        ports = list(list_ports.comports(True))
        arduino_port = None

        for p in ports:
            if p.serial_number == ARDUINO_SERIAL:
                arduino_port = p.device
                break

        if  arduino_port is None:
            print("error finding arduino")

        self.serial_connection = serial.Serial(arduino_port)

        # create services
        self.service = self.create_service(
            Fire, "fire", self.fire_callback)


    def fire_callback(self, request, response):
        gun_id = request.gun_id
        self.get_logger().info(f"Received request to fire gun with id {gun_id}")
        self.serial_connection.write(f"{gun_id}\n".encode(encoding='us-ascii'))
        raw_line = self.serial_connection.readline()
        self.get_logger().info("Received confirmation from arduino")
        return response

def main(args=None):
    rclpy.init(args=args)
    trigger = Trigger()
    rclpy.spin(trigger)
    rclpy.shutdown()


if __name__ == '__main__':
    main()