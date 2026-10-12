from setuptools import setup

package_name = 'p6_autonomous_mission'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/p6_autonomous_mission.launch.py']),
        ('share/' + package_name + '/config', ['config/nav2_offline.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dev',
    maintainer_email='dev@example.com',
    description='Offline P6 autonomous sampling mission',
    license='BSD-3-Clause',
    entry_points={
        'console_scripts': [
            'mission_coordinator = p6_autonomous_mission.mission_coordinator:main',
            'cave_simulator = p6_autonomous_mission.cave_simulator:main',
        ],
    },
)
