import os
from glob import glob

from setuptools import find_packages, setup

package_name = "hello"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/" + package_name, ["package.xml"]),
        (
            os.path.join("share", package_name, "launch"),
            glob(os.path.join("launch", "*launch.[pxy][yma]*")),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Christian Menard",
    maintainer_email="christian@xronos.com",
    description="Hello, World!",
    license="BSD",
    entry_points={
        "console_scripts": [
            "hello = hello.hello:main",
            "printer = hello.printer:main",
        ],
    },
)
