from setuptools import setup
from setuptools import setup
from setuptools import find_packages
import os

from glob import glob  # change for being able to read a file from a standard location

package_name = 'gps_point_at_carrot'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name + '/data', glob('data/*.txt')),  # change for opening the data file
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*launch.[pxy][yma]*')),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='adrian',
    maintainer_email='adrian23fernandez@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'vehicle_controller_point_at_carrot = gps_point_at_carrot.vehicle_controller_point_at_carrot:main',
        ],
    },
)
