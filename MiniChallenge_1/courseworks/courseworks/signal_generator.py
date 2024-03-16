import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

class My_Publisher(Node):
    def __init__(self):
        global time, result
        time = 0.0
        super().__init__('signal_generator_node')
        self.signal_publisher = self.create_publisher(Float32, 'signal', 10)
        self.time_publisher = self.create_publisher(Float32, 'time', 10)
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.get_logger().info('Signal generator node successfully initialized!!!')
        self.msg_signal = Float32()
        self.msg_time = Float32()

    def timer_callback(self):
        global time
        result = np.sin(time)
        self.msg_signal.data = result
        self.msg_time.data = time
        self.signal_publisher.publish(self.msg_signal)
        self.time_publisher.publish(self.msg_time)
        self.get_logger().info('Signal: {}'.format(self.msg_signal.data))
        self.get_logger().info('Time: {}'.format(self.msg_time.data))
        time += 0.1
        
def main(args=None):
    rclpy.init(args=args)
    m_p = My_Publisher()
    rclpy.spin(m_p)
    m_p.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()