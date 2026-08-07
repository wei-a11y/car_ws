from glob import glob
from setuptools import find_packages, setup


package_name = "delivery_robot_module_mocks"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(),
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
    description="Replaceable arm and cargo mock action servers.",
    license="BSD-3-Clause",
    entry_points={
        "console_scripts": [
            "module_mocks = delivery_robot_module_mocks.module_mocks:main",
        ],
    },
)
