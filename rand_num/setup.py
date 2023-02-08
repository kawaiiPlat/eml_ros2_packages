from setuptools import setup
import os
from glob import glob

package_name = 'rand_num'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*_launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='aditya',
    maintainer_email='apenumarti@ufl.edu',
    description='Random number generator',
    license='N/A',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rand_num = rand_num.rand_num:main',
        ],
    },
)
