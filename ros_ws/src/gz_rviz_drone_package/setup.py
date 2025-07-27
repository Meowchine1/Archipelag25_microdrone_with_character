from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'gz_rviz_drone_package'

def package_files(directory):
    paths = []
    for (path, _, filenames) in os.walk(directory):
        for filename in filenames:
            if not filename.endswith('generate_aruco_world.py'):
                full_path = os.path.join(path, filename)
                install_path = os.path.join('share', package_name, path)
                paths.append((install_path, [full_path]))
    return paths

data_files = [
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    (os.path.join('share', package_name, 'resource'), glob('resource/*rviz')),
]

# Добавляем вложенные gz_aruco_world/*
data_files += package_files('resource/gz_aruco_world')

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Meowchine1',
    maintainer_email='meowchine111@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'processes = gz_rviz_drone_package.processes:main',
            'visualizer = gz_rviz_drone_package.visualizer:main',
        ],
    },
)
