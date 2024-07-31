import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node


import xacro

def generate_launch_description():

    # verifica que se está usando tiempo de simulación
    use_sim_time = LaunchConfiguration('use_sim_time')

    # procesa el archivo URDF
    pkg_path = os.path.join(get_package_share_directory('york_joy_control'))
    xacro_file = os.path.join(pkg_path,'model','robot.urdf.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    
    # crea un lanzador para robot_description
    params = {'robot_description': robot_description_config.toxml(), 'use_sim_time': use_sim_time}
    launch_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    # crea un lanzador para rviz2
    launch_rviz2 = Node(
        package='rviz2',
        namespace='',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', [os.path.join(pkg_path, 'config', 'config_file.rviz')]]
    )
    

    # crea un lanzador para york_controller
    launch_york_controller= Node(
        package='york_joy_control',
        executable='york_controller',
        name='york_controller',
    )

    # crea un lanzador para joy_node
    launch_joy_node= Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
    )

    # crea un lanzador para joy_velocity_publisher
    launch_joy_velocity_publisher= Node(
        package='york_joy_control',
        executable='joy_velocity_publisher',
        name='joy_velocity_publisher',
    )

    # ejecuta los lanzadores!
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false',
            description='Use sim time if true'),
        launch_robot_state_publisher,
        launch_rviz2,
        launch_york_controller,
        launch_joy_node,
        launch_joy_velocity_publisher,

    ])