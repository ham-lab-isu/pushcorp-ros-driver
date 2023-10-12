from setuptools import setup
import os
from glob import glob

package_name = 'pushcorp_driver'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rosindustrial',
    maintainer_email='landongetting@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'afd_position_publisher = pushcorp_driver.afd_position_publisher:main',
            'afd_force_publisher = pushcorp_driver.afd_force_publisher:main'
        ],
    },
)
