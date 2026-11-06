from setuptools import find_packages, setup

package_name = "telepathic_management"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="OppaAI",
    maintainer_email="oppa.ai.org@gmail.com",
    description="Shared telepathic input and stimulus routing for AGi systems",
    license="GPL-3.0-only",
    entry_points={"console_scripts": ["telepathic_input = telepathic_management.input_node:main"]},
)
