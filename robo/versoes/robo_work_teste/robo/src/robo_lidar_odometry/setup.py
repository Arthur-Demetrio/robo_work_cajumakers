from glob import glob
import os

from setuptools import find_packages, setup


package_name = "robo_lidar_odometry"

setup(
    name=package_name,
    version="0.2.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        ("share/" + package_name, ["package.xml", "README.md"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools", "numpy"],
    zip_safe=True,
    maintainer="Otavio",
    maintainer_email="otavio@example.com",
    description="Odometria local 2D LIDAR-only por scan matching, sem encoders, EKF ou integracao de cmd_vel.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "lidar_odometry_node = "
            "robo_lidar_odometry.lidar_odometry_node:main",
        ],
    },
)
