from setuptools import setup
import os
from glob import glob

package_name = 'autocert_bringup'

setup(
    name=package_name,
    version='0.0.1',
    packages=[],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'config', 'robots'),
            glob('config/robots/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Gangotri',
    maintainer_email='kasturigangotri72@gmail.com',
    description='Launch orchestration for AutoCert',
    license='BSD-3-Clause',
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
