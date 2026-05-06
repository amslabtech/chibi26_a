from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch_ros.actions import LifecycleNode 
from launch.actions import ExecuteProcess, TimerAction
import os
from ament_index_python.packages import get_package_share_directory



def generate_launch_description():
    return LaunchDescription([
        Node(
            package='a_obstacle_detector',
            executable='a_obstacle_detector_node',
            # parameters=[{'use_sim_time': True}],
            parameters=['/home/user/ws/src/chibi26_a/obstacle_detector/config/param/obstacle_detector.yaml', {'use_sim_time': True}],
            # parameters=[{'use_sim_time': False}],
        ),
        Node(
            package='a_local_map_creator',
            executable='a_local_map_creator_node',
            # parameters=[{'use_sim_time': True}],
            parameters=['/home/user/ws/src/chibi26_a/local_map_creator/config/param/local_map_creator.yaml', {'use_sim_time': True}],
            # parameters=[{'use_sim_time': False}],
        ),
        Node(
            package='a_localizer',
            executable='a_localizer_node',
            # parameters=[{'use_sim_time': True}],
            parameters=['/home/user/ws/src/chibi26_a/localizer/config/param/localizer.yaml', {'use_sim_time': True}],
            # parameters=[{'use_sim_time': False}],
        ),
        Node(
            package='a_global_path_planner',
            executable='a_global_path_planner_node',
            parameters=[{'use_sim_time': True}],
            # parameters=['/home/user/ws/src/chibi26_a/local_goal_creator/config/param/local_goal_creator.yaml', {'use_sim_time': True}],
            # parameters=[{'use_sim_time': False}],
        ),
        Node(
            package='a_local_goal_creator',
            executable='a_local_goal_creator_node',
            parameters=[{'use_sim_time': True}],
            # parameters=['/home/user/ws/src/chibi26_a/local_goal_creator/config/param/local_goal_creator.yaml', {'use_sim_time': True}],
            # parameters=[{'use_sim_time': False}],
        ),
        Node(
            package='a_local_path_planner',
            executable='a_local_path_planner_node',
            # parameters=[{'use_sim_time': True}],
            parameters=['/home/user/ws/src/chibi26_a/local_path_planner/config/param/local_path_planner.yaml', {'use_sim_time': True}],
            # parameters=[{'use_sim_time': False}],
        ),
        
        Node(
          package='rviz2',
          executable='rviz2',
          arguments=['-d','./src/chibi26_a/bag/team_a/rviz_debag.rviz'],  #<-ファイルのある場所を変える必要あり
          parameters=[{'use_sim_time': True}],
        ),
        

        LifecycleNode(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            namespace='',
            output='screen',
            parameters=[{
                'yaml_filename': '/home/user/ws/src/chibi26_a/bag/team_a/map/a_map.yaml', #<-ファイルのある場所を要調整
                # 'yaml_filename': '/home/user/ws/map/map.yaml'
                'use_sim_time': True
            }]
        ),

        TimerAction(
            period=2.0,  # 秒数は状況に応じて調整（map_serverが準備できるくらい待つ）
            actions=[
                Node(
                    package='tf2_ros',
                    executable='static_transform_publisher',
                    # parameters=[{'use_sim_time': False}],
                    parameters=[{'use_sim_time': True}],
                    arguments=['0', '0', '0', '0', '0', '0', '1','/base_link', '/laser'],
                    ),
                ExecuteProcess(
                    # cmd=['ros2', 'bag', 'play', '/home/user/ws/src/chibi26_a/bag/team_a/rosbag2_2026_03_11-07_12_07', '--clock'], #<-bagファイルの指定を変える必要あり
                    cmd=['ros2', 'bag', 'play', '/home/user/ws/src/chibi26_a/bag/rosbag2_2026_05_05-08_56_58', '--clock'], 
                    output='screen'
                )
            ]
        ),
    
                
            
        TimerAction(
            period=2.0,
            actions=[
                ExecuteProcess(
                    cmd=['ros2', 'run', 'nav2_util', 'lifecycle_bringup', 'map_server'],
                    output='screen'
                )
            ]
        )

        

        
        
    ])