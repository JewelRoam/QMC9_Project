"""
Launch file for QMC9 simulation stack.
Starts CARLA interface, perception, planning, control, and cooperation nodes.

Usage:
    ros2 launch qmc9_bringup simulation.launch.py
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for QMC9 simulation."""
    
    # Launch arguments
    carla_host_arg = DeclareLaunchArgument(
        'carla_host',
        default_value='localhost',
        description='CARLA server host'
    )
    
    carla_port_arg = DeclareLaunchArgument(
        'carla_port',
        default_value='2000',
        description='CARLA server port'
    )
    
    map_name_arg = DeclareLaunchArgument(
        'map_name',
        default_value='Town03',
        description='CARLA map to load'
    )
    
    enable_cooperation_arg = DeclareLaunchArgument(
        'enable_cooperation',
        default_value='true',
        description='Enable multi-vehicle cooperation'
    )
    
    use_onnx_arg = DeclareLaunchArgument(
        'use_onnx',
        default_value='false',
        description='Use ONNX runtime for inference'
    )
    
    # CARLA ROS Interface Node
    carla_interface_node = Node(
        package='qmc9_simulation',  # Would need proper package setup
        executable='carla_ros_interface',
        name='carla_ros_interface',
        output='screen',
        parameters=[{
            'carla_host': LaunchConfiguration('carla_host'),
            'carla_port': LaunchConfiguration('carla_port'),
            'map_name': LaunchConfiguration('map_name'),
            'synchronous_mode': True,
            'fixed_delta_seconds': 0.05,
        }]
    )
    
    # Perception Node
    perception_node = Node(
        package='qmc9_perception',
        executable='perception_node',
        name='perception_node',
        output='screen',
        parameters=[{
            'model_path': 'model/yolov8n.pt',
            'confidence_threshold': 0.4,
            'use_onnx': LaunchConfiguration('use_onnx'),
            'onnx_model_path': 'model/yolo11n.onnx',
            'device': 'cuda',
            'publish_debug_image': True,
        }],
        remappings=[
            ('/sensors/camera/rgb/image_raw', '/sensors/camera/rgb/image_raw'),
            ('/sensors/camera/depth/image_raw', '/sensors/camera/depth/image_raw'),
        ]
    )
    
    # Planning Node
    planning_node = Node(
        package='qmc9_planning',
        executable='planning_node',
        name='planning_node',
        output='screen',
        parameters=[{
            'k_attractive': 1.0,
            'k_repulsive': 100.0,
            'd0': 20.0,
            'max_speed': 30.0,
            'planning_rate': 10.0,
        }]
    )
    
    # Control Node
    control_node = Node(
        package='qmc9_control',
        executable='control_node',
        name='control_node',
        output='screen',
        parameters=[{
            'pid_kp': 1.0,
            'pid_ki': 0.0,
            'pid_kd': 0.1,
            'max_steering_angle': 70.0,
            'control_rate': 20.0,
        }]
    )
    
    # Cooperation Node (conditional)
    cooperation_node = Node(
        package='qmc9_cooperation',
        executable='cooperation_node',
        name='cooperation_node',
        output='screen',
        condition=IfCondition(LaunchConfiguration('enable_cooperation')),
        parameters=[{
            'vehicle_id': 'ego_vehicle',
            'enable_cooperation': True,
            'communication_range': 100.0,
            'cooperation_mode': 'negotiated',
            'broadcast_rate': 10.0,
        }]
    )
    
    # RViz for visualization
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', 'config/qmc9.rviz'],
        output='screen'
    )
    
    return LaunchDescription([
        # Arguments
        carla_host_arg,
        carla_port_arg,
        map_name_arg,
        enable_cooperation_arg,
        use_onnx_arg,
        
        # Nodes
        carla_interface_node,
        
        # Delay perception until sensors are ready
        TimerAction(
            period=2.0,
            actions=[perception_node]
        ),
        
        # Delay planning until perception is ready
        TimerAction(
            period=3.0,
            actions=[planning_node]
        ),
        
        # Delay control until planning is ready
        TimerAction(
            period=4.0,
            actions=[control_node]
        ),
        
        # Cooperation node
        TimerAction(
            period=5.0,
            actions=[cooperation_node]
        ),
        
        # Visualization
        rviz_node,
    ])
