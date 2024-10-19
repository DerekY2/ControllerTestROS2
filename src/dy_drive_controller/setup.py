from setuptools import find_packages, setup
import os
from glob import glob
package_name = 'dy_drive_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='derek',
    maintainer_email='derek.yu869@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'joy_republisher = dy_drive_controller.joy_republisher:main',
            'drive_subscriber = dy_drive_controller.drive_subscriber:main'
        ],
    },  
    package_data={'dy_drive_controller':['constants.py']}
)
