from setuptools import setup
import os
from glob import glob

package_name = 'waypoint_follower_node'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    py_modules=['scripts.waypoint_follower'],
    data_files=[
        ('share/' + package_name, ['package.xml']),
        # Include all launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='Your Name',
    author_email='your_email@example.com',
    description='Waypoint follower node with automatic rosbag recording',
    license='License declaration',
    entry_points={
        'console_scripts': [
            'waypoint_follower = scripts.waypoint_follower:main',
        ],
    },
)
