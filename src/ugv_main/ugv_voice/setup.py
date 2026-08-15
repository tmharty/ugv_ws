from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'ugv_voice'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'),
            glob(os.path.join('config', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Travis Harty',
    maintainer_email='travisharty@gmail.com',
    description='Voice control for the UGV Beast: ear/brain/mouth nodes',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ear_node = ugv_voice.ear_node:main',
            'brain_node = ugv_voice.brain_node:main',
            'mouth_node = ugv_voice.mouth_node:main',
        ],
    },
)
