from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'vcs'

setup(
    name=package_name,
    version='0.1.2',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='OppaAI',
    maintainer_email='oppa.ai.org@gmail.com',
    description='Vital Circulatory System for AGi',
    license='GPL-3.0-only',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': ['vital_terminal_core = vcs.vtc:main'],
    },
)
