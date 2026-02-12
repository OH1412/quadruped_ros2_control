from setuptools import setup

package_name = "unitree_controller"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/sine_commander.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="user",
    maintainer_email="user@example.com",
    description="Python test node for Unitree GO-M8010-6 commands.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "sine_commander = unitree_controller.sine_commander:main",
            "keyboard_commander = unitree_controller.keyboard_commander:main",
            "fixed_angle_commander = unitree_controller.fixed_angle_commander:main",
        ],
    },
)
