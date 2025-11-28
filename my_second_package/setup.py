from setuptools import find_packages, setup

package_name = 'my_second_package'

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
    maintainer='cho',
    maintainer_email='samcho1588@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'lidar_simulator = my_second_package.lidar_simulator:main',
            'turtle_lidar_controller = my_second_package.turtle_lidar_controller:main',
            'lidar_node = my_second_package.lidar_node:main'
        ],
    },
)
