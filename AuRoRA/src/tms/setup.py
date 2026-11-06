from setuptools import find_packages, setup

package_name = 'tms'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'websockets'],
    zip_safe=True,
    maintainer='oppa-ai',
    maintainer_email='oppa.ai.org@proton.me',
    description='Telepathy Management System and authenticated HTTP API',
    license='GPL-3.0-only',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'api = tms.api:main',
            'tic = tms.tic:main',
        ],
    },
)
