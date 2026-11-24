from glob import glob
from setuptools import setup

setup(name='robo_virtual_wall', version='0.2.0', packages=['robo_virtual_wall'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/robo_virtual_wall']),
        ('share/robo_virtual_wall', ['package.xml', 'README.md', 'NAV2.md']),
        ('share/robo_virtual_wall/config', glob('config/*')),
        ('share/robo_virtual_wall/calibration', glob('calibration/*.json')),
        ('share/robo_virtual_wall/launch', glob('launch/*.launch.py'))],
    install_requires=['setuptools'], zip_safe=True,
    maintainer='Robo-WORK', maintainer_email='robo@example.com',
    description='Calibrated red/white tape keepout with stationary visible removal',
    license='Apache-2.0',
    entry_points={'console_scripts': [
        'virtual_wall = robo_virtual_wall.node:main',
        'tape_keepout = robo_virtual_wall.keepout_node:main',
        'validate_tape_calibration = robo_virtual_wall.calibrate:main']})
