import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    config = os.path.join(
        get_package_share_directory('reto_2'),
        'config',
        'params.yaml'
    )
    
    signal_generator_node = Node(
        package='reto_2',
        name='signal_generator_node',
        executable='signal_generator',
        output='screen',
        prefix='terminator --execute',
        parameters = [config]
    )
    
    signal_reconstruction_node = Node(
        package='reto_2',
        executable='signal_reconstruction',
        output='screen',
        prefix='terminator --execute',
    )

    rqt_graph_node = Node(
        package='rqt_graph',
        executable='rqt_graph',
        output='screen',
    )
    
    rqt_graph_plot = Node(
        package='rqt_plot',
        executable='rqt_plot',
        output='screen',
    )
    
    l_d = LaunchDescription([signal_generator_node, signal_reconstruction_node,
                             rqt_graph_node, rqt_graph_plot])

    

    return l_d
