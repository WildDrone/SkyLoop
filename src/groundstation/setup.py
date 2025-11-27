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
    description='TODO: Package description',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'groundstation_node = groundstation.gs:main',
            'perpetual_monitor_node = groundstation.perpetual_monitor_gui:main'
        ],
    },
)
