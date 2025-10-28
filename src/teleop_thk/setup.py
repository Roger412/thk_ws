from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'teleop_thk'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # required for ROS 2 package index
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        # include launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),

        # include rviz config files (optional but recommended)
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='roger',
    maintainer_email='joserogelioruiz12@gmail.com',
    description='Teleop launch setup with RViz and static TF publisher',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'teleop_thk = teleop_thk.teleop_thk:main',
        ],
    },
)
