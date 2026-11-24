from setuptools import setup
from glob import glob
package_name='robo_station_tracker_v041'
setup(name=package_name,version="0.4.1",packages=[package_name],
      data_files=[('share/ament_index/resource_index/packages',['resource/'+package_name]),
                  ('share/'+package_name,['package.xml']),
                  ('share/'+package_name+'/launch',glob('launch/*.launch.py')),
                  ('share/'+package_name+'/config',glob('config/*.yaml'))],
      install_requires=['setuptools'],zip_safe=True,maintainer='Geometry',maintainer_email='geometry@example.invalid',
      description='Read-only multi-hypothesis 2D LIDAR station candidate tracker',license='Apache-2.0',
      entry_points={'console_scripts':['station_tracker = robo_station_tracker_v041.tracker_node:main']})
