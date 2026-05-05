import os
from glob import glob
from setuptools import setup

package_name = "gp4_perception"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "config"), glob("config/*.*")),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
    ],
    install_requires=["setuptools", "numpy", "pyyaml", "scipy"],
    zip_safe=True,
    maintainer="user",
    maintainer_email="user@todo.todo",
    description="Perception stack for GP4 workcell.",
    license="Apache License 2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "calibration_service = gp4_perception.calibration:main_calibration_service",
            "tf_publisher = gp4_perception.calibration:main_tf_publisher",
            "scene_processor = gp4_perception.scene_processor:main",
        ],
    },
)
