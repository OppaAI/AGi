from setuptools import setup, find_packages
import os
from glob import glob

package_name = "scs"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "config"), glob(os.path.join("config", "*.yaml"))),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="OppaAI",
    maintainer_email="oppa.ai.org@gmail.com",
    description="Semantic Cognitive System — GRACE brain for AuRoRA",
    license="Apache-2.0",
    extras_require={"test": ["pytest"]},
    entry_points={
        "console_scripts": [
            "cnc = scs.cnc:main",
        ],
    },
)
