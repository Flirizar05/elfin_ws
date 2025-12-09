import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

def generate_launch_description():
    moveit2_pkg = get_package_share_directory('elfin3_ros2_moveit2')
    basic_api_pkg = get_package_share_directory('elfin_basic_api')

    moveit_ld = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(moveit2_pkg, 'launch', 'elfin3_moveit.launch.py')
        )
    )

    rviz_ld = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(moveit2_pkg, 'launch', 'elfin3_moveit_rviz.launch.py')
        )
    )

    basic_api_ld = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(moveit2_pkg, 'launch', 'elfin3_basic_api.launch.py')
        )
    )

    gui_ld = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(basic_api_pkg, 'launch', 'elfin_gui.launch.py')
        )
    )

    # -----------------------------------------------------------
    # ❌ Estos dos nodos ya NO los necesitamos → Se comentan
    # -----------------------------------------------------------

    # joint_state_aligner = Node(
    #     package='elfin_utils',
    #     executable='joint_state_aligner',
    #     name='joint_state_aligner',
    #     output='screen',
    #     emulate_tty=True,
    # )

    # trajectory_offset_shim = Node(
    #     package='elfin_utils',
    #     executable='trajectory_offset_shim',
    #     name='trajectory_offset_shim',
    #     output='screen',
    #     emulate_tty=True,
    # )

    # -----------------------------------------------------------
    # LaunchDescription final (sin aligner ni shim)
    # -----------------------------------------------------------

    return LaunchDescription([
        moveit_ld,
        TimerAction(period=5.0, actions=[rviz_ld]),
        TimerAction(period=10.0, actions=[basic_api_ld]),
        TimerAction(period=15.0, actions=[gui_ld]),

        # ❌ Ya no lanzamos estos
        # TimerAction(period=18.0, actions=[joint_state_aligner]),
        # TimerAction(period=21.0, actions=[trajectory_offset_shim]),
    ])
