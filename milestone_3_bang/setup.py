from setuptools import setup

package_name = 'milestone_3_bang'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Adrian F., Jarrod S., Mehron T.',
    maintainer_email='adrian23fernandez@gmail.com',
    description='Package that is a simple Bang Bang Controller for the Av1tenth Car',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'bb_controller = milestone_3_bang.bb_controller:main'
        ],
    },
)
