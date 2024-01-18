from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, ExecuteProcess,
                            LogInfo, RegisterEventHandler, TimerAction)
from launch.conditions import IfCondition
from launch.event_handlers import (OnExecutionComplete, OnProcessExit,
                                OnProcessIO, OnProcessStart, OnShutdown)
from launch.events import Shutdown
from launch.substitutions import (EnvironmentVariable, FindExecutable,
                                LaunchConfiguration, LocalSubstitution,
                                PythonExpression)


def generate_launch_description():
    asr_node = Node(
        package='simple_asr',
        executable='node',
        output= 'screen',
    )

    tts_service_server = Node(
        package='simple_tts',
        executable='server',
    )

    nlu_service_server = Node(
        package='simple_nlu',
        executable='server',
    )

    qa = Node(
        package='simple_qa',
        executable='qa',
    )


    return LaunchDescription([
        asr_node,
        tts_service_server,
        nlu_service_server,
        RegisterEventHandler(
            OnProcessExit(
                target_action=nlu_service_server,
                on_exit=[
                    LogInfo(msg=' Closing the turtlesim window'),
                    EmitEvent(event=Shutdown(
                        reason='Window closed'))
                ]
            )
        ),
        RegisterEventHandler(
            OnShutdown(
                on_shutdown=[LogInfo(
                    msg=['Launch was asked to shutdown: ',
                        LocalSubstitution('event.reason')]
                )]
            )
        ),
    ])