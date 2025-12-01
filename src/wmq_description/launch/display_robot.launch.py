import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch import substitutions
import os

def generate_launch_description():
    #获取默认的urdf路径
    urdf_package_path = get_package_share_directory('wmq_description')
    default_urdf_path = os.path.join(urdf_package_path, 'urdf', 'first_robot.urdf')
    default_rviz_config_path = os.path.join(urdf_package_path, 'config', 'display_robot_model.rviz')
    #声明一个urdf目录的参数，方便修改
    action_declare_arg_mode_path = launch.actions.DeclareLaunchArgument(
        name='model', default_value=str(default_urdf_path), description='加载的模型文件路径'
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

    #joint_state_publisher节点启动节点，声明可执行文件
    action_joint_state_publisher = launch_ros.actions.Node(
        package='joint_state_publisher',
        executable='joint_state_publisher'
    )

    action_rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', default_rviz_config_path]
    )

    return launch.LaunchDescription([
        action_declare_arg_mode_path,
        action_robot_state_publisher,
        action_joint_state_publisher,
        action_rviz_node
    ])