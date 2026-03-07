from setuptools import setup

package_name = 'livox_imu_converter'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rc_kfs',
    maintainer_email='rc_kfs@example.com',
    description='Scales Livox IMU linear acceleration from g to m/s^2 and republishes the message.',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'imu_converter_node = livox_imu_converter.imu_converter_node:main',
        ],
    },
)
