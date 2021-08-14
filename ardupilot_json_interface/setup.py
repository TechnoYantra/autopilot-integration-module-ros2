from setuptools import setup
import os
from glob import glob

package_name = 'ardupilot_json_interface'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('gazebo/*'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='chirag',
    maintainer_email='chiragmakwana02@gmail.com',
    description='Autopilot Integration Module',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ardupilot_json_interface_node = ardupilot_json_interface.ardupilot_json_interface_node:main'
            'twist_drive_node = ardupilot_json_interface.twist_drive:main'
        ],
    },
)
