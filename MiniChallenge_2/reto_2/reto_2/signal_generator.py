import numpy as np
import rclpy
from scipy import signal
import math
from rclpy.node import Node
from rclpy.parameter import Parameter
from sign_int.msg import Signal
from std_msgs.msg import Float32

class My_Publisher(Node):
    def __init__(self):
        super().__init__('signal_generator_node')
        self.declare_parameters(
            namespace='',
            parameters=[
                ('type_int', rclpy.Parameter.Type.INTEGER), 
                ('sine.time', rclpy.Parameter.Type.DOUBLE),
                ('sine.offset', rclpy.Parameter.Type.DOUBLE), 
                ('sine.frequency', rclpy.Parameter.Type.DOUBLE), 
                ('sine.amplitude', rclpy.Parameter.Type.DOUBLE),
                ('sine.phase', rclpy.Parameter.Type.DOUBLE),
                ('square.time', rclpy.Parameter.Type.DOUBLE),
                ('square.offset', rclpy.Parameter.Type.DOUBLE), 
                ('square.frequency', rclpy.Parameter.Type.DOUBLE), 
                ('square.amplitude', rclpy.Parameter.Type.DOUBLE),
                ('square.phase', rclpy.Parameter.Type.DOUBLE),
                ('sawtooth.time', rclpy.Parameter.Type.DOUBLE),
                ('sawtooth.offset', rclpy.Parameter.Type.DOUBLE), 
                ('sawtooth.frequency', rclpy.Parameter.Type.DOUBLE), 
                ('sawtooth.amplitude', rclpy.Parameter.Type.DOUBLE),
                ('sawtooth.phase', rclpy.Parameter.Type.DOUBLE),
                ('cosine.time', rclpy.Parameter.Type.DOUBLE),
                ('cosine.offset', rclpy.Parameter.Type.DOUBLE), 
                ('cosine.frequency', rclpy.Parameter.Type.DOUBLE), 
                ('cosine.amplitude', rclpy.Parameter.Type.DOUBLE),
                ('cosine.phase', rclpy.Parameter.Type.DOUBLE),
                ('half_square.time', rclpy.Parameter.Type.DOUBLE),
                ('half_square.offset', rclpy.Parameter.Type.DOUBLE), 
                ('half_square.frequency', rclpy.Parameter.Type.DOUBLE), 
                ('half_square.amplitude', rclpy.Parameter.Type.DOUBLE),
                ('half_square.phase', rclpy.Parameter.Type.DOUBLE)
            ]
        )
        self.signal_publisher = self.create_publisher(Float32, 'signal', 10)
        self.signal_params_publisher = self.create_publisher(Signal, 'signal_params', 10)
        self.time = 0
        self.signal_timer_period = 0.001
        self.signal_params_timer_period = 0.1
        self.signal_timer = self.create_timer(self.signal_timer_period, self.signal_timer_callback)
        self.params_timer = self.create_timer(self.signal_params_timer_period, self.signal_params_timer_callback)
        self.get_logger().info('Signal generator node successfully initialized!!!')
        self.msg_signal = Float32()
        self.msg_param = Signal()

    def signal_timer_callback(self):
        self.time += self.signal_timer_period
        self.type = self.get_parameter('type_int').get_parameter_value().integer_value
        if(self.type == 1):
            self.amplitude = self.get_parameter('sine.amplitude').get_parameter_value().double_value
            self.frequency = self.get_parameter('sine.frequency').get_parameter_value().double_value
            self.phase = self.get_parameter('sine.phase').get_parameter_value().double_value
            self.offset = self.get_parameter('sine.offset').get_parameter_value().double_value
            result = self.amplitude*np.sin(2*np.pi*self.time*self.frequency + self.phase) + self.offset
        elif(self.type == 2):
            self.amplitude = self.get_parameter('square.amplitude').get_parameter_value().double_value
            self.frequency = self.get_parameter('square.frequency').get_parameter_value().double_value
            self.phase = self.get_parameter('square.phase').get_parameter_value().double_value
            self.offset = self.get_parameter('square.offset').get_parameter_value().double_value
            result = self.amplitude*signal.square(2*np.pi*self.time*self.frequency + self.phase) + self.offset   
        elif(self.type == 3):
            self.amplitude = self.get_parameter('sawtooth.amplitude').get_parameter_value().double_value
            self.frequency = self.get_parameter('sawtooth.frequency').get_parameter_value().double_value
            self.phase = self.get_parameter('sawtooth.phase').get_parameter_value().double_value
            self.offset = self.get_parameter('sawtooth.offset').get_parameter_value().double_value
            result = self.amplitude*signal.sawtooth(2*np.pi*self.time*self.frequency + self.phase) + self.offset
        elif(self.type==4):
            self.amplitude = self.get_parameter('cosine.amplitude').get_parameter_value().double_value
            self.frequency = self.get_parameter('cosine.frequency').get_parameter_value().double_value
            self.phase = self.get_parameter('cosine.phase').get_parameter_value().double_value
            self.offset = self.get_parameter('cosine.offset').get_parameter_value().double_value
            result = abs(self.amplitude*np.cos(2*np.pi*self.time*self.frequency + self.phase)) + self.offset
        elif(self.type==5):
            self.amplitude = self.get_parameter('half_square.amplitude').get_parameter_value().double_value
            self.requency = self.get_parameter('half_square.frequency').get_parameter_value().double_value
            self.phase = self.get_parameter('half_square.phase').get_parameter_value().double_value
            self.offset = self.get_parameter('half_square.offset').get_parameter_value().double_value
            result = -self.amplitude*signal.square(np.pi/2*self.time*self.frequency + self.phase) + self.offset

        self.msg_signal.data = result
        self.signal_publisher.publish(self.msg_signal)
        self.get_logger().info('Signal: {}'.format(self.msg_signal.data))
    
    def signal_params_timer_callback(self):
        
        self.msg_param.type = self.type
        self.msg_param.frequency = self.frequency
        self.msg_param.amplitude = self.amplitude
        self.msg_param.time = self.time
        self.msg_param.offset = self.offset
        self.msg_param.phase = self.phase
        self.signal_params_publisher.publish(self.msg_param)
        self.get_logger().info('Signal type: {}'.format(self.msg_param.type))
        self.get_logger().info('Signal frequency: {}'.format(self.msg_param.frequency))
        self.get_logger().info('Signal amplitude: {}'.format(self.msg_param.amplitude))
        self.get_logger().info('Signal time: {}'.format(self.msg_param.time))
        self.get_logger().info('Signal offset: {}'.format(self.msg_param.offset))
        self.get_logger().info('Signal phase: {}'.format(self.msg_param.phase))
        
        
def main(args=None):
    rclpy.init(args=args)
    m_p = My_Publisher()
    rclpy.spin(m_p)
    m_p.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    
    