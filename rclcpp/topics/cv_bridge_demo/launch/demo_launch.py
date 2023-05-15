import launch
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='cv_bridge_demo',
            executable='cv_bridge_demo_bin',
            name='demo_name'),
  ])