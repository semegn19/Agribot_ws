from setuptools import setup
import os
from glob import glob

package_name = 'agribot_nav2_bringup'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        # ament index marker
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        # package.xml
        ('share/' + package_name, ['package.xml']),
        # launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        # params & maps
        (os.path.join('share', package_name, 'params'), glob('params/*')),
        (os.path.join('share', package_name, 'maps'), glob('maps/*')),
        
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='you',
    maintainer_email='you@example.com',
    description='Nav2 bringup for Agribot (params + launch wrapper)',
    license='Apache-2.0',
    tests_require=['pytest'],
)

