from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os.path

def generate_launch_description():
     ld = LaunchDescription()
     node_name = 'robo_cleaner_controller'

     print('[launch.py] - loading node ({0}) with no params'.format(node_name))

     node = Node(
          package = node_name,
          executable = node_name,
          output = 'screen',
          emulate_tty = True,
          parameters = None
     )

     ld.add_action(node)
     return ld