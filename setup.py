from setuptools import find_packages, setup
import os
from glob import glob


package_name = 'eureka_power_2'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Andrei Smirnov',
    maintainer_email='andrey040902@gmail.com',
    description='Nodes for power management of EUREKA Rover',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'battery_decoder = eureka_power_2.battery_decoder_uart:main',
        ],
    },
)
