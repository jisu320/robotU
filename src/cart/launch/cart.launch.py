import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node



def generate_launch_description():
    config_dir = os.path.join(get_package_share_directory('cart'),'config')
    rviz_config = os.path.join(config_dir,'mapping.rviz') #add
    urdf = os.path.join(config_dir,'RobotU.urdf') #add
    with open(urdf, 'r') as infp:#add
        robot_desc = infp.read()#add


    return LaunchDescription([

    Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='mapping_node',
        output='screen',
        arguments=['-configuration_directory', config_dir,'-configuration_basename','cartconfig.lua']
        ),
    Node(
        package='cartographer_ros',
        executable='occupancy_grid_node',
        name='occupancy_node',
        output='screen'

        ),

    #add
    Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        parameters=[
            {'use_gui': False},
            {'rate': 50}
        ]
        ),
    Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': False, 'robot_description': robot_desc}],
        arguments=[urdf]
        ),
    Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_node',
        arguments=['-d', rviz_config],
        output='screen'
        ),
    ])