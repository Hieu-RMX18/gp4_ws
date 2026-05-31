import os
from glob import glob
from setuptools import setup

package_name = "safety"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="hieu2",
    maintainer_email="hieu2@example.com",
    description="Safety layer for Yaskawa GP4 robot.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "safety_manager = safety.safety_manager:main",
        ],
    },
)
