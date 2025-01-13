import os
import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 获取当前包的路径
    package_share_directory = get_package_share_directory('web_socket')  # 替换为你包的名称
    
    # 拼接成完整的路径
    yaml_config_path = os.path.join(package_share_directory, 'config', 'poses.yaml')
    
    # 创建launch描述并启动节点
    return LaunchDescription([
        # 你可以在这里声明其他的 launch argument
        DeclareLaunchArgument(
            'yaml_config', default_value=yaml_config_path, description='Path to the YAML configuration file'
        ),
        
        # 定义并启动 ROS 2 节点
        Node(
            package='web_socket',          # ROS 2 package name
            executable='web_socket_node',   # ROS 2 executable (node)
            name='websocket_node',          # Name of the node
            output='screen',               # Output to screen
            parameters=[{
                'yaml_config': LaunchConfiguration('yaml_config'),
                 "host":"www.ykcel.com",
                 "port":"8088"
            }],
        ),
    ])
