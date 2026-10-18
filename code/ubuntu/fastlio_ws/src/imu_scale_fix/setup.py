from setuptools import setup
import os
from glob import glob

package_name = 'imu_scale_fix'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jerry',
    maintainer_email='jerry@example.com',
    description='Scale IMU acceleration from g to m/s^2',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'imu_scale_node = imu_scale_fix.imu_scale_node:main',
        ],
    },
)
