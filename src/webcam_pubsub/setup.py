from setuptools import setup

package_name = 'webcam_pubsub'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Daniel Riyavong',
    maintainer_email='danjan36@gmail.com',
    description='webcam_pubsub',
    license='Apache 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'webcam_pub = webcam_pubsub.webcam_pub:main',
        'webcam_sub = webcam_pubsub.webcam_sub:main',
        'dummy_pub = webcam_pubsub.dummy_pub:main'
        ],
    },
)
