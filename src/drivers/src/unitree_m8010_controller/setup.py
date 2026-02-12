from setuptools import setup

package_name = "unitree_m8010_controller"

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
    description="ROS2 Python node for Unitree GO-M8010-6 initialization, feedback, and control.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "m8010_controller = unitree_m8010_controller.m8010_controller:main",
        ],
    },
)
