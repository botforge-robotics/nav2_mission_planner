from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'nav2_mission_planner'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),


    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='chaitu',
    maintainer_email='nagachaitanya948@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'launch_manager = nav2_mission_planner.launch_manager:main',
        ],
    },
)
