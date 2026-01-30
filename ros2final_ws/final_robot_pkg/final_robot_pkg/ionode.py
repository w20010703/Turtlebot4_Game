#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import serial
from std_msgs.msg import String, Int32MultiArray

class ArduinoLEDNode(Node):

    def __init__(self):
        super().__init__('arduino_led_node')

        # ==== 修改成你的 serial port ====
        self.port = "/dev/ttyACM0"
        self.baud = 9600

        # 打開 Serial
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=0.1)
            self.get_logger().info(f"Connected to Arduino on {self.port}")
        except:
            self.get_logger().error("Cannot connect to Arduino serial port")
            raise

        # Publisher：RFID Tag
        self.tag_pub = self.create_publisher(String, 'rfid_tag', 10)

        # Publisher：Button Events
        self.btn_pub = self.create_publisher(String, 'button_event', 10)

        # Subscriber：接收顏色 (R,G,B)
        self.color_sub = self.create_subscription(
            Int32MultiArray,
            'led_color',
            self.color_callback,
            10
        )

        # Timer：每 10ms 讀一次 Arduino 回傳
        self.timer = self.create_timer(0.01, self.read_serial)

    # ----------------------------------------------------
    #   發送顏色給 Arduino
    # ----------------------------------------------------
    def color_callback(self, msg):
        if len(msg.data) != 3:
            self.get_logger().warn("Color must be [R,G,B]")
            return

        r, g, b = msg.data
        cmd = f"COLOR:{r},{g},{b}\n"
        self.ser.write(cmd.encode())
        self.get_logger().info(f"Send to Arduino → {cmd.strip()}")

    # ----------------------------------------------------
    #   讀取 Arduino 訊息
    # ----------------------------------------------------
    def read_serial(self):
        if self.ser.in_waiting > 0:
            line = self.ser.readline().decode().strip()

            if line == "":
                return

            self.get_logger().info(f"Arduino → {line}")

            # RFID 事件
            if line.startswith("TAG:"):
                msg = String()
                msg.data = line[4:]
                self.tag_pub.publish(msg)

            # 按鈕事件
            elif line.startswith("BTN:"):
                msg = String()
                msg.data = line[4:]
                self.btn_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ArduinoLEDNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

