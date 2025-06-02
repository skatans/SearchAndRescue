import os
import launch
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController

from launch_ros.actions import Node
from webots_ros2_driver.wait_for_controller_connection import WaitForControllerConnection


def generate_launch_description():
    package_dir = get_package_share_directory('my_package')

    webots = WebotsLauncher(
        world=os.path.join(package_dir, 'worlds', 'my_world.wbt'),
        ros2_supervisor=True
    )

    robot_description_path = os.path.join(package_dir, 'resource', 'mavic_webots.urdf')
    print(robot_description_path)
    mavic_driver = WebotsController(
        robot_name='Mavic_2_PRO',
        parameters=[
            {'robot_description': robot_description_path},
        ],
        respawn=True
    )

    # Give the /robot_description topic a dummy robot during initialization, turtlebot_driver will pass the correct description when it is ready.
    # This avoids the blocking behavior of the topic: if it receives nothing during initilization (turtlebot driver is not yet ready), it won't recover to receive
    # the correct turtlebot description when the driver is ready to pass it to the topic.
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': '<robot name=""><link name=""/></robot>'
        }],
    )

    # Turtlebot controller
    turtlebot_description_path = os.path.join(package_dir, 'resource', 'turtlebot_webots.urdf')
    ros2_control_params = os.path.join(package_dir, 'resource', 'ros2control.yml')
    mappings = [('/diffdrive_controller/cmd_vel', '/turtlebot/cmd_vel'), ('/diffdrive_controller/odom', '/turtlebot/odom')]
    turtlebot_driver = WebotsController(
        robot_name='TurtleBot3Burger',
        parameters=[
            {'robot_description': turtlebot_description_path,
             'set_robot_state_publisher': True},
            ros2_control_params
        ],
        remappings=mappings,
        respawn=True
    )

    # ROS control spawners for turtlebot hardware in simulation (wheels, motor)
    controller_manager_timeout = ['--controller-manager-timeout', '50']
    diffdrive_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        arguments=['diffdrive_controller'] + controller_manager_timeout,
    )
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        arguments=['joint_state_broadcaster'] + controller_manager_timeout,
    )
    ros_control_spawners = [diffdrive_controller_spawner, joint_state_broadcaster_spawner]

    # Wait for the simulation to be ready to start control spawner nodes: if started during initialization, the controls won't map correctly to turtlebot hardware
    waiting_nodes = WaitForControllerConnection(
        target_driver=turtlebot_driver,
        nodes_to_start= ros_control_spawners
    )

    return LaunchDescription([
        webots,
        webots._supervisor,
        robot_state_publisher,
        turtlebot_driver,
        mavic_driver,
        waiting_nodes,
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        )
    ])