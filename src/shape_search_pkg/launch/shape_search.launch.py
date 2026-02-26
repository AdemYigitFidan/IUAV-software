from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # MAVROS
        Node(
            package='mavros',
            executable='mavros_node',
            name='mavros',
            output='screen',
            parameters=[{
                'fcu_url': 'udp://:14550@',
                'gcs_url': '',
                'target_system_id': 1,
                'target_component_id': 1,
            }]
        ),
        
        # Gazebo-ROS2 Bridge (Kamera)
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='camera_bridge',
            arguments=[
                '/world/iris_runway/model/iris_with_gimbal/model/gimbal/link/pitch_link/sensor/camera/image@sensor_msgs/msg/Image@gz.msgs.Image'
            ],
            remappings=[
                ('/world/iris_runway/model/iris_with_gimbal/model/gimbal/link/pitch_link/sensor/camera/image', '/camera/image_raw')
            ],
            output='screen'
        ),
        
        # Shape Search Mission (bunu manuel çalıştıracağız)
        # Node(
        #     package='shape_search_pkg',
        #     executable='shape_search',
        #     name='shape_search_mission',
        #     output='screen'
        # ),
    ])