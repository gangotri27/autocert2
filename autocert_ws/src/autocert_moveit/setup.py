from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'autocert_moveit'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Gangotri',
    maintainer_email='kasturigangotri72@gmail.com',
    description='Dynamic MoveIt2 configuration for AutoCert',
    license='BSD-3-Clause',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'robot_description_loader = autocert_moveit.robot_description_loader:main',
        ],
    },
)
