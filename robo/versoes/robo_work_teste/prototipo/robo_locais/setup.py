from glob import glob

from setuptools import find_packages, setup

package_name = "robo_locais"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="marcos",
    maintainer_email="niviton.edu@gmail.com",
    description="PROTOTIPO: locais de interesse, planejador de objetivos e missao autonoma sobre o Nav2.",
    license="TODO",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "locais_node = robo_locais.locais_node:main",
            "missao_node = robo_locais.missao_node:main",
            "autonomo_node = robo_locais.autonomo_node:main",
            "ver_registro = robo_locais.registro:main_ver",
        ],
    },
)
