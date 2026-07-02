from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'groundstation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/static', glob('groundstation/static/*')),
    ],
    include_package_data=True,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='edr',
    maintainer_email='65714311+edouardrolland@users.noreply.github.com',
    description='ROS 2 groundstation for perpetual multi-drone monitoring with battery-relay missions.',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'perpetual_monitor_node = groundstation.perpetual_monitor_gui:main'
        ],
    },
)
