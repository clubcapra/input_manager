from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'input_manager'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        # Register the package with ament
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Install launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        # Install config files
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='Rescue robot input manager',
    license='MIT',
    entry_points={
        'console_scripts': [
            'input_manager = input_manager.main:main',
            'evdev_tester   = input_manager.evdev_code_tester:main',
        ],
    },
)
