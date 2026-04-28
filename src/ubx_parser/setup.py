from setuptools import find_packages, setup
from glob import glob

package_name = 'ubx_parser'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='phyllis-TANG',
    maintainer_email='1111phyllis1111@gmail.com',
    description='UBX protocol parser and .ubx replay node for u-blox GNSS receivers.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'replay_node = ubx_parser.replay_node:main',
            'ubx_to_csv = ubx_parser.ubx_to_csv:main',
        ],
    },
)
