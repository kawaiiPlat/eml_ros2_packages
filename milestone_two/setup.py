from setuptools import setup

package_name = 'milestone_two'

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
    description='Milestone 2 for EML4930',
    license='Death and Repudiation',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "milestone_two = milestone_two.laserToPC:main"
        ],
    },
)
