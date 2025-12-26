from setuptools import setup

package_name = 'AGi_core'

setup(
    name=package_name,
    version='0.1.0',
    packages=['vcs'],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='OppaAI',
    maintainer_email='oppa.ai.org@gmail.com',
    description='Vital Circulatory System modules for AGi',
    license='GPL-3.0-only',
    tests_require=['pytest'],
    # Add an entry point if this package were meant to be run directly
    entry_points={},
)