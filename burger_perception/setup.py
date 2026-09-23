"""Configuración de instalación del paquete burger_perception."""

from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'burger_perception'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Henry Roncancio',
    maintainer_email='henry.roncancio@unimilitar.edu.co',
    description='Localización AprilTag y razonamiento espacial de Burger-Cell.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'apriltag_localizer = burger_perception.apriltag_localizer:main',
            'gemini_spatial_reasoning_node = '
            'burger_perception.gemini_spatial_reasoning_node:main',
            'benchmark_gemini_apriltag = burger_perception.benchmark_gemini_apriltag:main',
        ],
    },
)
