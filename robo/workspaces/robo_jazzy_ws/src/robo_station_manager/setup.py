from setuptools import setup
from glob import glob

package_name = 'robo_station_manager'

setup(
    name=package_name,
    version='0.4.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Robo Geometry',
    maintainer_email='robo@example.com',
    description='Bancadas de teste, poses de aproximacao e preview de rotas Nav2 original e suavizada sem movimento',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'station_manager = robo_station_manager.station_manager_node:main',
            'station_cli = robo_station_manager.station_cli:main',
        ],
    },
)
