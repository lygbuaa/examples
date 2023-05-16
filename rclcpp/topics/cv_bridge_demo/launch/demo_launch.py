import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    package_share_directory = get_package_share_directory('cv_bridge_demo')
    package_rootdir = "{}/../../../../".format(package_share_directory)
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='cv_bridge_demo',
            executable='cv_bridge_demo_bin',
            name='demo_name',
            # log-level: DEBUG/INFO/WARN/ERROR/FATAL, do `colcon build` before launch.
            arguments=[package_rootdir, '--ros-args', '--log-level', 'INFO']
        ),
  ])