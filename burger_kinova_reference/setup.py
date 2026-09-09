"""Configuración de instalación del package burger_kinova_reference."""

from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'burger_kinova_reference'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'docs'), glob('docs/*.md')),
        (os.path.join('share', package_name, 'scripts'),
            glob('scripts/*.sh') + glob('scripts/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Henry Roncancio',
    maintainer_email='henry.roncancio@unimilitar.edu.co',
    description=(
        'Monitoreo, diagnóstico y comando articular seguro del Kinova Gen3 '
        'desde el proyecto burger_delivery, con subsistema de logging trazable.'
    ),
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'kinova_monitor = burger_kinova_reference.kinova_monitor:main',
            'safe_trajectory_client = '
            'burger_kinova_reference.safe_trajectory_client:main',
            'safe_sequence_client = '
            'burger_kinova_reference.safe_sequence_client:main',
        ],
    },
)
