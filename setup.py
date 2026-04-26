from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'multirobot_bringup'

setup(
    name=package_name,
    version='0.0.0',
    # Packages are located in the 'src' directory
    package_dir={'': 'src'},
    packages=find_packages(where='src'),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Add launch files
        ('share/' + package_name + '/launch', glob('launch/*.py') + glob('launch/*.xml')),
        # Add config files
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        # Add rviz configurations
        ('share/' + package_name + '/rviz', glob('rviz/*.rviz')),
        # Add urdf/xacro files
        ('share/' + package_name + '/urdf', glob('urdf/*.xacro') + glob('urdf/*.urdf')),
        # Add world files
        ('share/' + package_name + '/worlds', glob('worlds/*.sdf') + glob('worlds/*.world')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='aitor',
    maintainer_email='aitoribi100@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        ],
    },
)
