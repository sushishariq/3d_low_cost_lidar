from setuptools import find_packages, setup

package_name = 'lidar_simulation'

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
    maintainer='shariq',
    maintainer_email='sj260.24@kgpian.iitkgp.ac.in',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
		'scan_simulator = lidar_simulation.scan_simulator:main',
		'encoder_simulator = lidar_simulation.encoder_simulator:main',
		'reconstruction_node = lidar_simulation.reconstruction_node:main',
        ],
    },
)
