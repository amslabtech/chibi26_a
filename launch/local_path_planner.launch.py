import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # パッケージのシェアディレクトリを取得
    package_dir = get_package_share_directory('team_local_path_planner')
    
    # YAMLファイルのパスを自動作成
    config_file = os.path.join(package_dir, 'config', 'dwa_params.yaml')

    return LaunchDescription([
        Node(
            package='team_local_path_planner',
            executable='team_local_path_planner_node',
            name='local_path_planner_node',
            parameters=[config_file],
            output='screen',
        )
    ])