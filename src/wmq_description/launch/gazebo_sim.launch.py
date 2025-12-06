import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import os

def generate_launch_description():
    #获取功能包的share路径
    urdf_package_path = get_package_share_directory('wmq_description')
    default_xacro_path = os.path.join(urdf_package_path, 'urdf', 'wmqbot', 'wmqbot.urdf.xacro')
    # default_rviz_config_path = os.path.join(urdf_package_path, 'config', 'display_robot_model.rviz')
    default_gazebo_world_path = os.path.join(urdf_package_path, 'world', 'new_house.world')
    #声明一个urdf目录的参数，方便修改
    action_declare_arg_mode_path = launch.actions.DeclareLaunchArgument(
        name='model', default_value=str(default_xacro_path), description='加载的模型文件路径'
    )

    #通过文件路径，获取内容，并转换成参数值对象，以供传入robot_state_publisher
    #通过launch.substitutions.Command执行一句cat的命令行指令
    #launch.substitutions.LaunchConfiguration('model')会将指令中传入的参数替换model中的模型路径
    #最后通过cat/xacro将模型中的数据打印出来传入substitutions_command_result
    substitutions_command_result = launch.substitutions.Command(['xacro ', launch.substitutions.LaunchConfiguration('model')])
    
    #捕获命令行中的输出并将其转换成参数值
    robot_description_value = launch_ros.parameter_descriptions.ParameterValue(substitutions_command_result, 
        value_type=str)
    
    #robot_state_publisher节点启动节点，传入一个参数robot_description_value（本质为urdf文件的内容）
    action_robot_state_publisher = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description':robot_description_value}]
    )

    # #joint_state_publisher节点启动节点，声明可执行文件
    # action_joint_state_publisher = launch_ros.actions.Node(
    #     package='joint_state_publisher',
    #     executable='joint_state_publisher'
    # )

    # action_rviz_node = launch_ros.actions.Node(
    #     package='rviz2',
    #     executable='rviz2',
    #     arguments=['-d', default_rviz_config_path]
    # )

    action_launch_gazebo = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            [get_package_share_directory('gazebo_ros'), '/launch', '/gazebo.launch.py']
        ),
        launch_arguments={'world': default_gazebo_world_path, 'verbose': 'true'}.items()
    )

    action_spawn_entity = launch_ros.actions.Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', '/robot_description', '-entity', 'fishbot']
    )

    action_load_joint_state_controller = launch.actions.ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', 'fishbot_joint_state_broadcaster', '--set-state', 'start'],
        output='screen'
    )
    # 选择加载两轮差速控制器
    action_load_effort_controller = launch.actions.ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', 'fishbot_effort_controller', '--set-state', 'start'],
        output='screen'
    )

    action_load_diff_driver_controller = launch.actions.ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', 'fishbot_diff_drive_controller', '--set-state', 'start'],
        output='screen'
    )

    return launch.LaunchDescription([
        action_declare_arg_mode_path,
        action_robot_state_publisher,
        action_launch_gazebo,
        action_spawn_entity,
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=action_spawn_entity,
                on_exit=[action_load_joint_state_controller]
            )
        ),
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=action_load_joint_state_controller,
                on_exit=[action_load_diff_driver_controller]
            )
        ),
    ])