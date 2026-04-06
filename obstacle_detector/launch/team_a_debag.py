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
            package='team_obstacle_detector',
            executable='team_obstacle_detector_node',
            parameters=[{'use_sim_time': True}],
            #  parameters=[{'use_sim_time': False}],
        ),
        # Node(
        #     package='teamb_local_map_creator',
        #     executable='teamb_local_map_creator_node',
        #     parameters=[{'use_sim_time': True}],
        #     # parameters=[{'use_sim_time': False}],
        # ),
        Node(
            package='a_localizer',
            executable='team_localizer_node',
            parameters=[{'use_sim_time': True}],
            #  parameters=[{'use_sim_time': False}],
        ),
        Node(
            package='team_global_path_planner',
            executable='team_global_path_planner_node',
            parameters=[{'use_sim_time': True}],
            #  parameters=[{'use_sim_time': False}],
        ),
        Node(
            package='team_local_goal_creator',
            executable='team_local_goal_creator_node',
            parameters=[{'use_sim_time': True}],
            #  parameters=[{'use_sim_time': False}],
        ),
        Node(
            package='team_local_path_planner',
            executable='team_local_path_planner_node',
            parameters=[{'use_sim_time': True}],
            #  parameters=[{'use_sim_time': False}],
        ),
        
        Node(
          package='rviz2',
          executable='rviz2',
          arguments=['-d','./src/chibi25_b/local_path_planner/launch/teamb_rviz2_debag.rviz'],  #<-ファイルのある場所を変える必要あり
          parameters=[{'use_sim_time': True}],
        ),
        

        LifecycleNode(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            namespace='',
            output='screen',
            parameters=[{
                'yaml_filename': '/home/user/ws/src/chibi26_a/bag/map/a_map.yaml', #<-ファイルのある場所を要調整
                # 'yaml_filename': '/home/user/ws/map/map.yaml'
                'use_sim_time': True
            }]
        ),

        TimerAction(
            period=1.0,  # 秒数は状況に応じて調整（map_serverが準備できるくらい待つ）
            actions=[
                Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            parameters=[{'use_sim_time': True}],
            arguments=['0', '0', '0', '0', '0', '0', '1','/base_link', '/laser'],
        ),
        ExecuteProcess(
            cmd=['ros2', 'bag', 'play', '/home/user/ws/bagfiles/team_c/rosbag2_2026_03_11-05_14_01', '--clock'], #<-bagファイルの指定を変える必要あり
          output='screen'
        )
            ]
        ),
    
                
            
        TimerAction(
            period=2.0,  # 秒数は状況に応じて調整（map_serverが準備できるくらい待つ）
            actions=[
                Node(
                    package='nav2_util',
                    executable='lifecycle_bringup',
                    name='map_server_lifecycle',
                    output='screen',
                    arguments=['map_server']
                )
            ]
        )

        

        
        
    ])