import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'closed_loop'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), 
            glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='boris',
    maintainer_email='boris@arizona.edu',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'hello_cl = closed_loop.hello_cl:main',
            'motion_command = closed_loop.motion_command:main',
            'wall_detector = closed_loop.wall_detector:main',
        ],
    },
)
