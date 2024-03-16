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
                ('time_float', rclpy.Parameter.Type.DOUBLE),
                ('offset_float', rclpy.Parameter.Type.DOUBLE), 
                ('frequency_float', rclpy.Parameter.Type.DOUBLE), 
                ('amplitude_float', rclpy.Parameter.Type.DOUBLE)
            ]
        )
        self.signal_publisher = self.create_publisher(Float32, 'signal', 10)
        self.signal_params_publisher = self.create_publisher(Signal, 'signal_params', 10)
        signal_timer_period = 0.001
        signal_params_timer_period = 0.1
        self.signal_timer = self.create_timer(signal_timer_period, self.signal_timer_callback)
        self.params_timer = self.create_timer(signal_params_timer_period, self.signal_params_timer_callback)
        self.get_logger().info('Signal generator node successfully initialized!!!')
        self.msg_signal = Float32()
        self.msg_param = Signal()

    def signal_timer_callback(self):
        
        type = self.get_parameter('type_int').get_parameter_value().integer_value
        frequency = self.get_parameter('frequency_float').get_parameter_value().double_value
        amplitude = self.get_parameter('amplitude_float').get_parameter_value().double_value
        time = self.get_parameter('time_float').get_parameter_value().double_value
        offset = self.get_parameter('offset_float').get_parameter_value().double_value
        if(type == 1):
            result = amplitude*np.sin(2*np.pi*time*frequency) + offset
        elif(type == 2):
            result = amplitude*signal.square(2*np.pi*time*frequency) + offset   
        elif(type == 3):
            result = amplitude*signal.sawtooth(2*np.pi*time*frequency) + offset
        elif(type==4):
            result = abs(amplitude*np.cos(2*np.pi*time*frequency)) + offset
        elif(type==5):
            result = -amplitude*signal.square(np.pi/2*time*frequency) + offset

        self.msg_signal.data = result
        self.signal_publisher.publish(self.msg_signal)
        self.get_logger().info('Signal: {}'.format(self.msg_signal.data))
        param_time = Parameter('time_float', Parameter.Type.DOUBLE, time + 0.001)
        self.set_parameters([param_time])
    
    def signal_params_timer_callback(self):
        
        self.msg_param.type = self.get_parameter('type_int').get_parameter_value().integer_value
        self.msg_param.frequency = self.get_parameter('frequency_float').get_parameter_value().double_value
        self.msg_param.amplitude = self.get_parameter('amplitude_float').get_parameter_value().double_value
        self.msg_param.time = self.get_parameter('time_float').get_parameter_value().double_value
        self.msg_param.offset = self.get_parameter('offset_float').get_parameter_value().double_value
        self.signal_params_publisher.publish(self.msg_param)
        self.get_logger().info('Signal type: {}'.format(self.msg_param.type))
        self.get_logger().info('Signal frequency: {}'.format(self.msg_param.frequency))
        self.get_logger().info('Signal amplitude: {}'.format(self.msg_param.amplitude))
        self.get_logger().info('Signal time: {}'.format(self.msg_param.time))
        self.get_logger().info('Signal offset: {}'.format(self.msg_param.offset))
        
        
def main(args=None):
    rclpy.init(args=args)
    m_p = My_Publisher()
    rclpy.spin(m_p)
    m_p.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    
    