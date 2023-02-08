from setuptools import setup

package_name = 'basic_pub_sub'

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
    maintainer='Jarrod Sanders',
    maintainer_email='jarrod.sanders@ufl.edu',
    description='A package to demonstrate a basic pub/sub for Autonomous Robotics',
    license='MIT or D&R',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'talker = basic_pub_sub.my_pub:main',
            'listener = basic_pub_sub.my_sub:main',
        ],
    },
)
