import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'my_rc_car'

# Only include real launch files (exclude __pycache__ and any dirs)
launch_files = [f for f in glob('launch/*.py') if os.path.isfile(f)]

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', [f'resource/{package_name}']),
        (f'share/{package_name}', ['package.xml']),
        (f'share/{package_name}/launch', launch_files),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='antonyf',
    maintainer_email='anthony060206@icloud.com',
    description='RC car nodes (Python) with rclpy and python-can',
    license='MIT',
    entry_points={
        'console_scripts': [
            'rc_controller = my_rc_car.rc_controller:main',
            'drive_control_canbus = my_rc_car.drive_control_canbus:main',
        ],
    },
)
