from setuptools import setup
from glob import glob
package_name='robo_corridor_guard'
setup(
    name=package_name,version='0.1.0',packages=[package_name],
    data_files=[('share/ament_index/resource_index/packages',['resource/'+package_name]),
                ('share/'+package_name,['package.xml']),
                ('share/'+package_name+'/launch',glob('launch/*.launch.py'))],
    install_requires=['setuptools'],zip_safe=True,
    maintainer='Geometry team',maintainer_email='geometry@example.invalid',
    description='Read-only corridor geometry observer and simulated command guard for Nav2 Jazzy.',
    license='MIT',
    entry_points={'console_scripts':['corridor_observer=robo_corridor_guard.node:main']}
)
