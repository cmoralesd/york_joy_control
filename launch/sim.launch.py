import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.actions import RegisterEventHandler, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from pathlib import Path

import xacro

def generate_launch_description():

    # Check if we're told to use sim time
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)

    # Process the URDF file
    pkg_path = os.path.join(get_package_share_directory('york_joy_control'))
    xacro_file = os.path.join(pkg_path,'model','robot.urdf.xacro')
    robot_description = xacro.process_file(xacro_file)

    # Set gazebo sim resource path
    gazebo_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[
            os.path.join(pkg_path, 'config', 'worlds'), ':' +
            str(Path(pkg_path).parent.resolve())
            ]
        )
    
    arguments = LaunchDescription([
                DeclareLaunchArgument('world', default_value='test_world',
                          description='GazeboSim World'),
           ]
    )
    
    launch_gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('ros_gz_sim'), 'launch'), '/gz_sim.launch.py']),
                launch_arguments=[
                    ('gz_args', [LaunchConfiguration('world'),
                                 '.sdf',
                                 ' -v 4',
                                 ' -r']
                    )
                ]
             )
    
    gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-string', robot_description.toprettyxml(indent='  '),
                   '-x', '0.0',
                   '-y', '0.0',
                   '-z', '0.025941',
                   '-R', '0.0',
                   '-P', '0.0',
                   '-Y', '0.0',
                   '-name', 'york',
                   '-allow_renaming', 'false'],
    )

    # Create a robot_state_publisher node
    params = {'robot_description': robot_description.toxml(), 'use_sim_time': use_sim_time}
    launch_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    # Create a rviz node
    launch_rviz2= Node(
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
    


    # Launch!
    return LaunchDescription([
        # Iniciamos york_controller sólo despues de que el modelo se despliega en gazebo,
        # para así garantizar que la simulación se mantiene sincronizada la odometría.
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=gz_spawn_entity,
                on_exit=[launch_york_controller],
            )
        ),
        # todos estos nodos se inician sin un orden específico
        launch_robot_state_publisher,
        launch_rviz2,
        gazebo_resource_path,
        arguments,
        launch_gazebo,
        gz_spawn_entity,
        launch_joy_node,
        launch_joy_velocity_publisher

    ])