from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'shape_search_pkg'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), 
            glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), 
            glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Adem Yiğit Fidan',
    maintainer_email='your_email@example.com',
    description='ArduPilot drone shape detection and search',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
    'console_scripts': [
        'drone_controller = shape_search_pkg.drone_controller:main',
        'shape_detector = shape_search_pkg.shape_detector:main',
        'shape_search = shape_search_pkg.shape_search_mission:main',
        'yolo_detector = shape_search_pkg.yolo_detector:main',  # BU SATIRI EKLE
    ],
},
)