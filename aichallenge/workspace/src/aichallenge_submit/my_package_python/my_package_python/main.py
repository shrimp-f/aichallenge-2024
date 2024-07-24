import rclpy  # ROS2のPythonモジュール
from rclpy.node import Node
from std_msgs.msg import String # トピック通信に使うStringメッセージ型をインポート

class SampleClass(Node):
    def __init__(self):
        super().__init__('sample_node') # ROS1でいうrospy.init_node('node_name')
        self.pub = self.create_publisher(String,'/sample_msg', 10)# publisherの宣言
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello, world: %d' % self.i
        self.pub.publish(msg)
        self.i += 1

def main():
    rclpy.init()
    node = SampleClass()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()