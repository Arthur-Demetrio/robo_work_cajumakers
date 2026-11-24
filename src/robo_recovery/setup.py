from setuptools import find_packages, setup

package_name = 'robo_recovery'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(
        exclude=['test']
    ),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name]
        ),
        (
            'share/' + package_name,
            ['package.xml']
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='marcos',
    maintainer_email='marcos@todo.todo',
    description=(
        'Recuperacoes mecanum para o robo movel'
    ),
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mecanum_escape_node = '
            'robo_recovery.mecanum_escape_node:main',
        ],
    },
)
