from setuptools import find_packages, setup

package_name = 'vsc'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='OppaAI',
    maintainer_email='oppa.ai.org@gmail.com',
    description='Vitality System Core for AGi',
    license='GPLv3',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'heartbeat = vsc.heartbeat:main',
        ],
    },
)
