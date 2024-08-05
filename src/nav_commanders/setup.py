import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'nav_commanders'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='erik',
    maintainer_email='erikcaell@gmail.com',
    description='Python Simple Commanders to control high level nav2 logic',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'example_nav_to_pose = nav_commanders.nav_to_pose:main',
            'gps_commander = nav_commanders.nav_to_gps_coords:main',
            'follow_aruco_marker = nav_commanders.follow_vision_target:main',
            'follow_circ_trails = nav_commanders.follow_circ_trails:main'
        ],
    },
)
