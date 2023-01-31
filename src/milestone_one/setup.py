from setuptools import setup

package_name = 'milestone_one'

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
    description='Milestone 1 for EML4930',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'listener = milestone_one.quotient_publisher:main',
        ],
    },
)
