from setuptools import setup
import os
from glob import glob

package_name = 'robotaksi_interface'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sarp',
    maintainer_email='robotaksi@todo.todo',
    description='Robotaksi PC to Arduino Serial Bridge and Kinematics Node',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'bridge_node = robotaksi_interface.bridge_node:main',
        ],
    },
)
