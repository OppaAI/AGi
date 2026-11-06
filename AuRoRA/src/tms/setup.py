import os
from pathlib import Path
from setuptools import find_packages, setup

package_name = 'tms'

package_root = Path(__file__).resolve().parent
repo_root = package_root.parents[2]
web_teleops_console_root = repo_root / 'interface' / 'web_teleops_console'

web_teleops_console_files = [path for path in web_teleops_console_root.rglob('*') if path.is_file()]

data_files = [
    (
        'share/' + package_name + '/web_teleops_console/' +
        str(Path(path).parent.relative_to(web_teleops_console_root)),
        [os.path.relpath(path, package_root)],
    )
    for path in web_teleops_console_files
]

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        *data_files,
    ],
    install_requires=['setuptools', 'fastapi', 'uvicorn', 'websockets'],
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
            'web_teleops_console = tms.web_teleops_console:main',
            'tic = tms.tic:main',
        ],
    },
)
