from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'final_robot_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # Package index
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),

        # Package manifest
        ('share/' + package_name, ['package.xml']),

        # Install launch files
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='julia',
    maintainer_email='julia@example.com',
    description='Robot FSM + Move + IO + TTS integration package',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'movenode = final_robot_pkg.movenode:main',
            'ionode = final_robot_pkg.ionode:main',
            'fsmnode = final_robot_pkg.fsmnode:main',
            'ttsnode = final_robot_pkg.ttsnode:main',
        ],
    },
)

