import numpy as np
import rclpy
from scipy import signal
from rclpy.node import Node
from sign_int.msg import Signal
from std_msgs.msg import Float32


class My_Subscriber(Node):
    def __init__(self):
        super().__init__('signal_reconstruction_node')
        self.sub = self.create_subscription(Signal, 'signal_params', self.reconstruction_callback, 10)
        self.resignal_publisher = self.create_publisher(Float32, 'signal_reconstructed', 10)
        resignal_timer_period = 0.1
        self.resignal_timer = self.create_timer(resignal_timer_period, self.resignal_timer_callback)
        self.get_logger().info('Signal reconstruction node successfully initialized!!!')
        self.msg_resignal = Float32()
        self.time = None

    def reconstruction_callback(self, msg):
        self.type = msg.type
        self.amplitude = msg.amplitude
        self.time = msg.time
        self.offset = msg.offset
        self.frequency = msg.frequency   
        self.phase = msg.phase
    
    def resignal_timer_callback(self):
        #If there is a signal, then it recontructs it
        if(self.time is not None):
            if(self.type == 1):
                result = self.amplitude*np.sin(2*np.pi*self.time*self.frequency + self.phase) + self.offset
            elif(self.type == 3):
                result = self.amplitude*signal.sawtooth(2*np.pi*self.time*self.frequency + self.phase) + self.offset
            elif(self.type == 2):
                result = self.amplitude*signal.square(2*np.pi*self.time*self.frequency + self.phase) + self.offset 
            elif(self.type == 4):
                result = abs(self.amplitude*np.cos(2*np.pi*self.time*self.frequency + self.phase)) + self.offset
            elif(self.type == 5):
                result = -self.amplitude*signal.square(np.pi/2*self.time*self.frequency + self.phase) + self.offset  
            self.msg_resignal.data = result
            self.resignal_publisher.publish(self.msg_resignal)
            self.get_logger().info('Signal reconstructed: {}'.format(self.msg_resignal.data))
        #if there is no signal, the result is 0
        elif():
            result = 0


def main(args=None):
    rclpy.init(args=args)
    m_s = My_Subscriber()
    rclpy.spin(m_s)
    m_s.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    