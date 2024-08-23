from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'bbot_firmware'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # (os.path.join('share', package_name, 'arduino_codes'), glob('arduino_codes/*')),
        (os.path.join('share', package_name, 'bbot_firmware'), glob('bbot_firmware/*'))

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='vedh',
    maintainer_email='vedhamshbode@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "serial_transmitter = bbot_firmware.Simple_serial_transmitter:main"
        ],
    },
)
