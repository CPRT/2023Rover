from setuptools import find_packages, setup

package_name = 'camera_processing'
submodules = [
    'camera_processing/zed_helper_files', 
    'camera_processing/HSVImageExplore/image_colour_processing',
    'camera_processing/zed_viewers/cv_viewer',
    'camera_processing/zed_viewers/ogl_viewer']

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']) + submodules,
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='erik',
    maintainer_email='erikcaell@gmail.com',
    description='Contains nodes to process camera data',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'aruco_markers = camera_processing.aruco_markers:main',
            'display_image_locally = camera_processing.display_image_locally:main',
            'zed_node = camera_processing.zed_node:main',
        ],
    },
)
