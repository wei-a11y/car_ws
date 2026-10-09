import xml.etree.ElementTree as ET

from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("car_urdf", package_name="arm_moveit_config").to_moveit_configs()

    # The shared URDF declares Gazebo hardware for the base. This standalone
    # demo uses mock hardware instead, retaining the stationary wheel states
    # without changing the description used by the Gazebo/navigation launch.
    robot = ET.fromstring(moveit_config.robot_description["robot_description"])
    for system in robot.findall("ros2_control"):
        plugin = system.find("hardware/plugin")
        if plugin is not None and plugin.text.strip() == "gazebo_ros2_control/GazeboSystem":
            system.set("name", "DemoBaseSystem")
            plugin.text = "mock_components/GenericSystem"
    moveit_config.robot_description["robot_description"] = ET.tostring(robot, encoding="unicode")

    return generate_demo_launch(moveit_config)
