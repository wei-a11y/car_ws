from glob import glob
from setuptools import find_packages, setup


package_name = "car_chassis_task"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "README.md"]),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="user",
    maintainer_email="user@example.com",
    description="Chassis task action, localization supervision, and side docking.",
    license="BSD-3-Clause",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "chassis_task_server = car_chassis_task.chassis_task_server:main",
            "localization_supervisor = car_chassis_task.localization_supervisor:main",
            "velocity_arbiter = car_chassis_task.velocity_arbiter:main",
        ],
    },
)
