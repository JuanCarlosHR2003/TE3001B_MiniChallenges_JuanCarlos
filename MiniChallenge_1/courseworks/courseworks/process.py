import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

class Process_Node(Node):
    def __init__(self):
        global result, time
        time = None        
        result = None
        super().__init__('process_node')
        self.signal_sub = self.create_subscription(Float32, 'signal', self.signal_callback, 10)
        self.time_sub = self.create_subscription(Float32, 'time', self.time_callback, 10)
        
        self.proc_sig_publisher = self.create_publisher(Float32, 'proc_signal', 10)
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.msg_proc_sig = Float32()
        
        self.get_logger().info('Process node successfully initialized!!!')

    def time_callback(self, msg):
        global time
        self.get_logger().info('Time: {}'.format(msg.data))
        time = msg.data
        
    def signal_callback(self, msg):
        global result
        self.get_logger().info('Signal: {}'.format(msg.data))
        result = msg.data
        
    def timer_callback(self):
        global result, proc_sig, time
        if((time is not None) and (result is not None) ):
            proc_sig = result*np.cos(np.pi) + np.cos(time)*np.sin(np.pi)
            proc_sig = (proc_sig + 1)/2 
            self.msg_proc_sig.data = proc_sig
            self.proc_sig_publisher.publish(self.msg_proc_sig)
            self.get_logger().info('Processed signal: {}'.format(self.msg_proc_sig.data))
        
        
def main(args=None):
    rclpy.init(args=args)
    n_p = Process_Node()
    rclpy.spin(n_p)
    n_p.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()