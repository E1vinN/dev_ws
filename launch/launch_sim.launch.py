import os
import sys
#package_path = 'dev_ws/src/my_bot/lauch'
#sys.path.append(package_path)

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node




def generate_launch_description():


    # Include the robot_state_publisher launch file, provided by our own package. Force sim time to be enabled
    # !!! MAKE SURE YOU SET THE PACKAGE NAME CORRECTLY !!!

    package_name='my_bot' #<--- CHANGE ME

    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','rsp.launch.py'
                )]), launch_arguments={'use_sim_time': 'true'}.items()
    )
    
        # Define the path to your custom world file
    custom_world_path = os.path.join(
        get_package_share_directory('my_bot'),
        'worlds',
        'obstacles.world'  # <-- CHANGE ME TO YOUR WORLD FILE NAME
    )

    # Include the Gazebo launch file, provided by the gazebo_ros package
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
                    launch_arguments={'world': custom_world_path}.items()
             )

    # Run the spawner node from the gazebo_ros package. The entity name doesn't really matter if you only have a single robot.
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'my_bot'],
                        output='screen')
                        
    #algorithm_node = Node(package='my_bot',  # Change this to your package name
     #   executable='algorithm_node',  # Change this to your executable name
      #  output='screen'
    #)
    
   # obstacle_avoidance = Node(package=package_path,
    #    executable='obstacle_avoidance',  
     #   output='screen',
    #)



    # Launch them all!
    return LaunchDescription([
        rsp,
        gazebo,
        spawn_entity,
       # obstacle_avoidance,
     #   algorithm_node,
    ])
