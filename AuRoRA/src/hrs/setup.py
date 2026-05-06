from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'hrs'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join("share", "hrs", "launch"), glob("launch/*.py")),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='OppaAI',
    maintainer_email='oppa.ai.org@gmail.com',
    description='Homeostatic Regulation System for AuRoRA (Autonomous Rover Robotics Assistant)',
    license='GPL-3.0-only',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'ras = hrs.ras:main',
        ],
    },
)
