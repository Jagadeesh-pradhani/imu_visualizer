from setuptools import find_packages, setup
import os
from glob import glob
package_name = 'imu_visualizer'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.*')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='liros',
    maintainer_email='liros@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'imu_viz = imu_visualizer.imu_visualizer_node:main',
        ],
    },
)
