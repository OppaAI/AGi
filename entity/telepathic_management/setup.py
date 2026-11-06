import os
from pathlib import Path
from setuptools import find_packages, setup

package_name = "telepathic_management"
package_root = Path(__file__).resolve().parent
shared_files = [path for path in package_root.rglob("*") if path.is_file() and "__pycache__" not in path.parts]
shared_data = [
    ("share/" + package_name + "/" + str(path.parent.relative_to(package_root)), [os.path.relpath(path, package_root)])
    for path in shared_files if path.is_relative_to(package_root / "telepathic_management" / "telepathic_console" / "frontend") or path.is_relative_to(package_root / "telepathic_management" / "telepathic_console" / "backend")
]

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        *shared_data,
    ],
    install_requires=["setuptools", "fastapi", "uvicorn", "websockets"],
    zip_safe=True,
    maintainer="OppaAI",
    maintainer_email="oppa.ai.org@gmail.com",
    description="Shared telepathic input and stimulus routing for AGi systems",
    license="GPL-3.0-only",
    entry_points={"console_scripts": ["telepathic_input = telepathic_management.telepathic_input:main", "telepathic_console = telepathic_management.telepathic_console.server:main"]},
)
