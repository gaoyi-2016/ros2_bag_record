from launch import LaunchDescription  
from launch_ros.actions import Node    

from ament_index_python.packages import get_package_share_directory  
import os

from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():     
  package_dir = get_package_share_directory('ros2_bag_record')
  config_path = os.path.join(package_dir, 'config','config.yaml')

  return LaunchDescription([          
    Node(
      package="ros2_bag_record",
      executable="main",
      output="screen",
       arguments=[config_path]
    )
  ])