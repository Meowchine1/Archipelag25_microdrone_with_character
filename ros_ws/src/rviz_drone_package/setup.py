import os
from glob import glob
from setuptools import setup
package_name = 'rviz_drone_package'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
    ('share/ament_index/resource_index/packages',
        ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    (os.path.join('share', package_name, 'launch'), glob('launch/*launch.[pxy][yma]*')),
    (os.path.join('share', package_name, 'resource'), glob('resource/*.rviz'))
],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Meowchine1',
    maintainer_email='meowchine111@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'movenment_control = rviz_drone_package.movenment_control:main',
            'processes = rviz_drone_package.processes:main',
            'state_control = rviz_drone_package.state_control:main',
            'visualizer = rviz_drone_package.visualizer:main',
        ],
    },
)
