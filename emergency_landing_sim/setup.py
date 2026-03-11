from glob import glob
import os
from setuptools import setup

package_name = 'emergency_landing_sim'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*')),
        (os.path.join('share', package_name, 'meshes'), glob('meshes/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nus-iss',
    maintainer_email='your_email@example.com',
    description='Emergency landing simulation package',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
           'rotor_spin_node = emergency_landing_sim.rotor_spin_node:main', 'emergency_landing_node = emergency_landing_sim.emergency_landing_node:main',
            'gazebo_cmdvel_bridge = emergency_landing_sim.gazebo_cmdvel_bridge:main',
        ],
    },
)
