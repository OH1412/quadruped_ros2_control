from setuptools import setup

package_name = "unitree_command_publisher"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="user",
    maintainer_email="user@example.com",
    description="Simple ROS2 publisher for UnitreeCommand.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "unitree_command_publisher = unitree_command_publisher.publisher_node:main",
        ],
    },
)
