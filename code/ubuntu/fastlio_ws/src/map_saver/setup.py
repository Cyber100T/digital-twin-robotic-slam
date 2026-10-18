from setuptools import setup

package_name = 'map_saver'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jerry',
    maintainer_email='jerry@example.com',
    description='Save Laser_map PointCloud2 to PCD',
    license='MIT',
    entry_points={
        'console_scripts': [
            'save_laser_map = map_saver.save_laser_map:main',
        ],
    },
)
