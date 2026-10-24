from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'chassis_serial'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lm',
    maintainer_email='lm@todo.todo',
    description='订阅 /cmd_vel 差速换算后经串口发下位机；接收编码器回传积分发布 /odom',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'chassis_serial_node = chassis_serial.chassis_serial_node:main',
        ],
    },
)
