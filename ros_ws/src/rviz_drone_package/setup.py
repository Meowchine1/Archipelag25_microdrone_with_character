from setuptools import find_packages, setup

package_name = 'rviz_drone_package'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
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
            'movenment_control = rviz_drone_package:movenment_control',
            'processes = rviz_drone_package:processes',
            'state_control = rviz_drone_package:state_control',
            'visualizer = rviz_drone_package:visualizer',
        ],
    },
)
