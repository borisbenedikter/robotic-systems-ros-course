from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'pub_sub'

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
            'pub_sub_hello_world = pub_sub.pub_sub_hello_world:main',
            'pub_sub_talker = pub_sub.publisher_member_function:main',
            'pub_sub_listener = pub_sub.subscriber_member_function:main',
        ],
    },
)
