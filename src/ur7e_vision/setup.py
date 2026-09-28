from glob import glob
import os

from setuptools import find_packages, setup


package_name = 'ur7e_vision'


setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='wenqin',
    maintainer_email='wenqin@todo.todo',
    description='RGB-D color cube detection and 3D localization for the UR7e workcell.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'color_cube_detector = ur7e_vision.color_cube_detector:main',
        ],
    },
)
