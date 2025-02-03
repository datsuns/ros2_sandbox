
from launch import LaunchDescription
from launch.actions import RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessStart
from launch_ros.actions import LifecycleNode
from launch.actions import ExecuteProcess


def generate_launch_description():
    # ライフサイクルノードを定義
    lifecycle_node = LifecycleNode(
        package='my_lifecycle_node',
        executable='lifecycle_node',
        name='my_lifecycle_node',
        namespace='',
        output='screen'
    )

    # configure ステートに遷移させる
    configure_event_handler = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=lifecycle_node,
            on_start=[
                TimerAction(
                    period=0.0,  # 起動後2秒待機してから "configure" を送信
                    actions=[
                        ExecuteProcess(
                            cmd=['ros2', 'lifecycle', 'set', '/my_lifecycle_node', 'configure'],
                            output='screen'
                        )
                    ]
                )
            ]
        )
    )

    # activate ステートに遷移させる
    activate_event_handler = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=lifecycle_node,
            on_start=[
                TimerAction(
                    period=0.0,  # 起動後4秒待機してから "activate" を送信
                    actions=[
                        ExecuteProcess(
                            cmd=['ros2', 'lifecycle', 'set', '/my_lifecycle_node', 'activate'],
                            output='screen'
                        )
                    ]
                )
            ]
        )
    )

    # LaunchDescriptionにノードとイベントハンドラを追加
    return LaunchDescription([
        lifecycle_node,
        configure_event_handler,
        activate_event_handler
    ])
