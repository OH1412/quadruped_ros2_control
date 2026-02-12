from setuptools import setup

package_name = "unitree_fixed_commander"

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
    description="Fixed 12-axis command publisher for Unitree motors.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "fixed_commander = unitree_fixed_commander.fixed_commander:main",
        ],
    },
)
