"""Configuración de instalación del paquete burger_navigation."""

from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'burger_navigation'

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
    description='Navegación de los carritos y acción de entrega de Burger-Cell.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'delivery_pose_server = burger_navigation.delivery_pose_server:main',
            'car_tf_coupler = burger_navigation.car_tf_coupler:main',
        ],
    },
)
