from glob import glob
import os

from setuptools import setup

package_name = 'tzb_sim'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tzb',
    maintainer_email='tzb@example.com',
    description='Synthetic RGB-D scene for the tzb arm pipeline',
    license='MIT',
    entry_points={
        'console_scripts': [
            'blue_block_scene = tzb_sim.blue_block_scene:main',
            'serial_bridge_sim = tzb_sim.serial_bridge_sim:main',
        ],
    },
)
