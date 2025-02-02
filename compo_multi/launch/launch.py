import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    """Generate launch description with multiple components."""
    container = ComposableNodeContainer(
        name='my_container',
        namespace='',
        package='rclcpp_components',
        #executable='component_container',
        executable='component_container_mt',
        composable_node_descriptions=[
                ComposableNode(
                    package='compo_multi',
                    plugin='MinimalSubscriber',
                    name='sub'),
        ],
        output='screen',
        arguments=['--executor-args', '--num-threads 4']
    )

    return launch.LaunchDescription([container])
