from setuptools import setup

package_name = 'robo_progressive_spin'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robo_geometry',
    maintainer_email='robo@localhost',
    description='Progressive collision-aware Nav2 Spin action wrapper',
    license='Apache-2.0',
    entry_points={'console_scripts': [
        'progressive_spin_node = robo_progressive_spin.progressive_spin_node:main',
    ]},
)
