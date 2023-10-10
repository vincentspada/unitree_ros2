import launch
import launch.actions
import launch.substitutions
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            'rname',
            default_value='go1',
            description='Robot name'
        ),
        launch.actions.DeclareLaunchArgument(
            'robot_name',
            default_value=launch.substitutions.LaunchConfiguration('rname'),
            description='Robot name parameter'
        ),
        # Add your nodes here, for example:
        # launch_ros.actions.Node(
        #     package='unitree_controller',
        #     executable='unitree_servo',
        #     name='unitree_servo',
        #     output='screen',
        #     parameters=[{'robot_name': launch.substitutions.LaunchConfiguration('robot_name')}]
        # )
    ])
