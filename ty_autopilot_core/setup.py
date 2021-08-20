from setuptools import setup
import os
from glob import glob
package_name = 'ty_autopilot_core'
mavutils = 'ty_autopilot_core/mavutils'
node = 'ty_autopilot_core/node'
services = 'ty_autopilot_core/services'


setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, mavutils, node, services],
    data_files=[('share/ament_index/resource_index/packages',['resource/' + package_name]),
                ('share/' + package_name, ['package.xml']),
                (os.path.join('share', package_name), glob('launch/*.py')),
                (os.path.join('share', package_name), glob('config/*'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='chirag',
    maintainer_email='chiragmakwana02@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'vehicle_controller_node = ty_autopilot_core.vehicle_controller:main',
            'cmd_vel_mux_selector_node = ty_autopilot_core.node.cmd_vel_mux_selector:main',
        ],
    },
)
