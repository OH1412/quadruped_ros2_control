from setuptools import setup

package_name = "unitree_feedback_reader"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name, ["README.md"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="user",
    maintainer_email="user@example.com",
    description="Unitree GO-M8010-6 feedback reader.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "feedback_reader = unitree_feedback_reader.feedback_reader:main",
        ],
    },
)
