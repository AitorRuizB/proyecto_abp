from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'proyecto_abp'

setup(
    name=package_name,
    version='0.0.0',
    # Ahora usamos la forma estándar de ROS 2 para encontrar el código Python
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Las rutas relativas ahora funcionan perfectamente con la nueva estructura
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py') + glob('launch/*.xml')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.xacro') + glob('urdf/*.urdf')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.sdf') + glob('worlds/*.world')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='aitor',
    maintainer_email='aitoribi100@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'differential_drive = proyecto_abp.differentialDrive:main',
            'camera_subscriber = proyecto_abp.cameraSubscriber:main',
        ],
    },
)