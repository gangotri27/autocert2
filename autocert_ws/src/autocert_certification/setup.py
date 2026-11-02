from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'autocert_certification'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'templates'), glob('templates/*.tex')),
    ],
    install_requires=['setuptools', 'urdf_parser_py', 'numpy', 'scipy',],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='kasturigangotri72@gmail.com',
    description='ISO 9283 certification report generation',
    license='BSD-3-Clause',
    extras_require={
        'test': [
            'pytest',
        ],
        'pdf': [
            'reportlab',
        ],
        'latex': [
            'matplotlib',  # For placeholder image
            'Pillow',      # For image handling
        ],
    },
    entry_points={
        'console_scripts': [
            'certification_node = autocert_certification.certification_node:main',
        ],
    },
)
