from glob import glob
from setuptools import find_packages, setup

package_name = "robo_lidar"

setup(
    name=package_name,
    version="0.2.2",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Otavio",
    maintainer_email="otavio@example.com",
    description="Filtro modular do LiDAR para ROS 2.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "scan_filter_node = robo_lidar.scan_filter_node:main",
        ],
    },
)
