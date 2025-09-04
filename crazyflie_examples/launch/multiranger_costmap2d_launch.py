import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, EmitEvent, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.events import matches_action
from launch_ros.actions import Node, LifecycleNode
from launch_ros.events.lifecycle import ChangeState
from launch_ros.event_handlers import OnStateTransition
from lifecycle_msgs.msg import Transition


def generate_launch_description():

    costmap_config_path = os.path.join(
        get_package_share_directory('crazyflie_examples'),
        'config/costmap_drone.yaml'
    )
    rviz_gui_config_path = os.path.join(
        get_package_share_directory('crazyflie_examples'),
        'config/local_costmap_only.rviz' # drone_costmap, local_costmap_only
    )


    costmap_node = LifecycleNode(
        package='nav2_costmap_2d',
        executable='nav2_costmap_2d',
        name='costmap',
        namespace='costmap',
        output='screen',
        parameters=[
            costmap_config_path,
            {
                'use_sim_time': False,
                'global_frame': 'cf231',
                'robot_base_frame': 'cf231',
                'obstacle_layer.scan.topic': '/cf231/scan',
                'static_layer.enabled': False,
                # 'width': 10,
                # 'height': 10,
                # 'rolling_window': True,
                # 'track_unknown_space': False,
                # 'static_layer.enabled': False,
                # 'plugins': ['obstacle_layer', 'inflation_layer']
            }
        ],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz",
        arguments=["-d", rviz_gui_config_path],
        parameters=[{"use_sim_time": True}],
        # condition=IfCondition(rviz),
    )

    costmap_node_bringup = [
        EmitEvent(
            event=ChangeState(
                lifecycle_node_matcher=matches_action(costmap_node),
                transition_id=Transition.TRANSITION_CONFIGURE,
            )
        ),
        RegisterEventHandler(
            OnStateTransition(
                target_lifecycle_node=costmap_node,
                goal_state='inactive',
                entities=[
                    EmitEvent(
                        event=ChangeState(
                            lifecycle_node_matcher=matches_action(costmap_node),
                            transition_id=Transition.TRANSITION_ACTIVATE,
                        )
                    )
                ],
            )
        ),
    ]

    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [
                        os.path.join(
                            get_package_share_directory('crazyflie'), 'launch'
                        ),
                        '/launch.py',
                    ]
                ),
                launch_arguments={
                    'backend': 'cflib',
                    'gui': 'false',
                    'teleop': 'false',
                    'mocap': 'false',
                }.items(),
            ),
            costmap_node,
            *costmap_node_bringup,
            rviz_node,
        ]
    )
