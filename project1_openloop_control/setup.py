import os
from glob import glob
from setuptools import setup

package_name = 'project1_openloop_control'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Adrian F., Jarrod S., Mehron T.',
    maintainer_email='adrian23fernandez@gmail.com',
    description='Package that convers joy data into ackermann data',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'control_handler = project1_openloop_control.control_handler:main',
        ],
    },
)
